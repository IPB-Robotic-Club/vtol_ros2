import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_pid_config, get_takeoff_config
import time
import json
import math
import cv2
import numpy as np

LOG_PATH       = "/home/pilot/workspace/mission_centering.log"
CSV_LOG_PATH   = "/home/pilot/workspace/centering_data.csv"


class PIDController:
    """
    PID Controller dengan derivative low-pass filter untuk mencegah D-term spike.
    Update seharusnya dipanggil dari satu timer periodik yang konsisten,
    BUKAN dari callback event yang memiliki jitter timing.
    """
    def __init__(self, kp, ki, kd, max_out, d_filter_alpha=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        # Alpha untuk low-pass filter pada D-term (0=sangat smooth, 1=no filter)
        self.d_filter_alpha = d_filter_alpha

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_output = 0.0
        self.filtered_derivative = 0.0  # State low-pass filter
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_dt = 0.0

    def update(self, error, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            self.last_output = 0.0
            self.last_dt = 0.0
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return self.last_output

        # Abaikan update jika dt terlalu kecil (kurang dari 30ms) untuk mencegah ledakan D-term dari jitter ROS
        if dt < 0.03:
            return self.last_output

        self.last_dt = dt

        # Proportional
        self.p_term = self.kp * error

        # Integral dengan Anti-windup
        self.integral += error * dt
        max_integral = self.max_out / (self.ki if self.ki != 0 else 1.0)
        self.integral = max(min(self.integral, max_integral), -max_integral)
        self.i_term = self.ki * self.integral

        # Derivative dengan low-pass filter untuk meredam spike dari jitter dt
        raw_derivative = (error - self.last_error) / dt
        
        # Clamp raw derivative to prevent wild kicks from sudden marker shifts
        max_derivative = 2.0
        raw_derivative = max(min(raw_derivative, max_derivative), -max_derivative)
        # Low-pass: filtered = alpha * raw + (1-alpha) * prev_filtered
        self.filtered_derivative = (
            self.d_filter_alpha * raw_derivative
            + (1.0 - self.d_filter_alpha) * self.filtered_derivative
        )
        self.d_term = self.kd * self.filtered_derivative

        # Output total dengan clamp
        output = self.p_term + self.i_term + self.d_term
        output = max(min(output, self.max_out), -self.max_out)

        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_output = 0.0
        self.filtered_derivative = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_dt = 0.0


def apply_smooth_deadzone(u_raw, deadzone, band=2.0):
    """
    Kompensasi deadzone RC dengan transisi linier kontinu di dekat nol.
    Mencegah osilasi bang-bang yang terjadi saat pakai hard cut-off.
    """
    if abs(u_raw) < band:
        # Interpolasi kemiringan curam tapi kontinu (tidak ada loncatan)
        return u_raw * ((deadzone + band) / band)
    else:
        return u_raw + math.copysign(deadzone, u_raw)


class MissionCenteringNode(VtolBaseNode):
    def __init__(self):
        # Buka file log khusus di workspace (selalu ditimpa saat inisialisasi)
        try:
            self.log_file = open(LOG_PATH, "w")
            self.log_file.write("=== LOG MISI CENTERING DIMULAI ===\n")
            self.log_file.flush()
        except Exception as e:
            print(f"Warning: Gagal membuka file log di {log_path}: {e}")
            self.log_file = None

        # Aktifkan timer publish RC bawaan base class (10Hz)
        super().__init__('mission_centering_node', enable_rc_loop=True)

        # Load parameters
        self.pid_params = get_pid_config()
        self.max_override = self.pid_params['max_override']
        self.error_threshold = self.pid_params['error_threshold']
        self.centering_duration = self.pid_params['centering_duration']
        self.marker_lost_timeout = self.pid_params['marker_lost_timeout']
        takeoff_cfg = get_takeoff_config()
        self.takeoff_altitude = takeoff_cfg['takeoff_altitude']

        # Altitude hold aktif: P-controller pada RC3 (throttle) untuk mempertahankan
        # target_altitude selama centering, karena LOITER di SITL tidak selalu hold altitude
        # secara sempurna (THR_MID bias, momentum, dll).
        self.alt_hold_enabled   = self.pid_params.get('hold_altitude', True)
        self.kp_altitude        = self.pid_params.get('kp_altitude', 30.0)
        self.max_alt_correction = self.pid_params.get('max_throttle_correction', 80)
        # hover_base: titik tengah RC3 yang sesuai dengan hover drone di SITL.
        # Di SITL ArduPilot, RC3=1500 (LOITER center) seringkali BUKAN hover point asli
        # akibat THR_MID bias. Nilai ini dikalibrasi dari observasi empiris.
        # Default=1500 (disable offset). Set ke ~1545-1550 jika drone masih turun.
        self.hover_base         = self.pid_params.get('hover_base', 1500)
        self.get_logger().info(
            f"[AltHold] enabled={self.alt_hold_enabled} | hover_base={self.hover_base} | "
            f"kp={self.kp_altitude} | max_correction=±{self.max_alt_correction}"
        )

        # Inisialisasi PID dengan low-pass filter pada D-term (alpha=0.4 untuk keseimbangan smoothing & delay)
        # max_out dikurangi deadzone_bias agar output final tidak melebihi max_override
        deadzone_bias = float(self.pid_params.get('deadzone_bias', 35.0))
        pid_max_raw = max(5.0, self.max_override - deadzone_bias)
        self.pid_roll = PIDController(
            self.pid_params['kp_roll'], self.pid_params['ki_roll'],
            self.pid_params['kd_roll'], pid_max_raw, d_filter_alpha=0.4
        )
        self.pid_pitch = PIDController(
            self.pid_params['kp_pitch'], self.pid_params['ki_pitch'],
            self.pid_params['kd_pitch'], pid_max_raw, d_filter_alpha=0.4
        )
        # Yaw PID: koreksi heading drone agar axis kamera sejajar dengan marker
        deadzone_bias_yaw = self.pid_params.get('deadzone_bias_yaw', 25.0)
        max_yaw_raw = self.pid_params.get('max_yaw_override', 30) - deadzone_bias_yaw
        self.pid_yaw = PIDController(
            self.pid_params.get('kp_yaw', 5.0), self.pid_params.get('ki_yaw', 0.2),
            self.pid_params.get('kd_yaw', 8.0), max_yaw_raw, d_filter_alpha=0.4
        )
        self.yaw_error_threshold = self.pid_params.get('yaw_error_threshold', 0.08)
        self.enable_yaw_alignment = self.pid_params.get('enable_yaw_alignment', False)

        self.write_log(
            f"LOADED PARAMETERS: enable_yaw_alignment={self.enable_yaw_alignment}, "
            f"kp_roll={self.pid_roll.kp}, ki_roll={self.pid_roll.ki}, "
            f"kd_roll={self.pid_roll.kd}, max_override={self.max_override}, "
            f"d_filter_alpha={self.pid_roll.d_filter_alpha} | "
            f"kp_yaw={self.pid_yaw.kp}, yaw_threshold={self.yaw_error_threshold:.3f}rad"
        )

        # Buka CSV log terstruktur untuk analisis post-flight
        try:
            self.csv_file = open(CSV_LOG_PATH, "w")
            self.csv_file.write(
                "src,wall_time,loop_iter,vision_ts,frame_no,frame_w,frame_h,"
                "center_x,center_y,norm_ex,norm_ey,dist,alt,yaw_err,phase,"
                "pid_dt,p_roll,i_roll,d_roll,raw_roll,final_roll,rc_roll,"
                "p_pitch,i_pitch,d_pitch,raw_pitch,final_pitch,rc_pitch,"
                "p_yaw,i_yaw,d_yaw,raw_yaw,final_yaw,rc_yaw,"
                "rc_throttle,stable_dur\n"
            )
            self.csv_file.flush()
        except Exception as e:
            self.write_log(f"[WARN] Gagal membuka CSV log: {e}")
            self.csv_file = None

        # State untuk data terbaru dari vision (diupdate oleh detection_callback)
        # PID TIDAK dijalankan di sini — hanya menyimpan state
        self.last_detection_time = 0.0
        self.last_vision_timestamp = 0.0   # Timestamp dari pesan vision itu sendiri
        self.last_vision_frame_no = -1
        self.last_vision_frame_w = 640.0
        self.last_vision_frame_h = 480.0
        self.marker_detected = False
        self.first_detection_made = False
        self.norm_error_x = 0.0   # Error X terbaru dari vision
        self.norm_error_y = 0.0   # Error Y terbaru dari vision
        self.yaw_error    = 0.0   # Yaw error dari pose_rvec[2] (radian)
        self.has_yaw_measurement = False  # Flag apakah pose_rvec valid telah diterima
        self.last_raw_center_x = 0.0
        self.last_raw_center_y = 0.0
        self.last_frame_count = -1
        self.last_center_x = -1.0
        self.last_center_y = -1.0
        self.loop_iter = 0

        self.stable_start_time = None
        self.centered = False
        self.centering_active = False
        # Phase control: 'YAW_ALIGN' dulu jika enable_yaw_alignment=True,
        # sebaliknya langsung ke 'CENTERING' (Roll/Pitch saja)
        self.phase = 'YAW_ALIGN' if self.enable_yaw_alignment else 'CENTERING'
        self.yaw_aligned = not self.enable_yaw_alignment

        # Berlangganan topik deteksi dari vtol_vision
        self.detection_sub = self.create_subscription(
            String,
            '/vtol/aruco/detection',
            self.detection_callback,
            10
        )

        self.write_log(f"Mission Centering Node Siap! CSV log -> {CSV_LOG_PATH}")

    def write_log(self, message):
        self.get_logger().info(message)
        if self.log_file:
            try:
                self.log_file.write(f"[{time.strftime('%H:%M:%S')}] {message}\n")
                self.log_file.flush()
            except Exception:
                pass

    def get_current_time(self):
        """Mendapatkan waktu saat ini menggunakan clock ROS (mendukung sim_time)."""
        return self.get_clock().now().nanoseconds / 1e9

    def write_warn(self, message):
        self.get_logger().warn(message)
        if self.log_file:
            try:
                self.log_file.write(f"[{time.strftime('%H:%M:%S')}] [WARN] {message}\n")
                self.log_file.flush()
            except Exception:
                pass

    def destroy_node(self):
        if hasattr(self, 'log_file') and self.log_file:
            try:
                self.log_file.write("=== LOG MISI CENTERING BERAKHIR ===\n")
                self.log_file.close()
            except Exception:
                pass
        if hasattr(self, 'csv_file') and self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        super().destroy_node()

    def detection_callback(self, msg):
        """
        Hanya menyimpan state terbaru dari vision pipeline.
        PID TIDAK diupdate di sini untuk menghindari jitter dt dari callback timing.
        """
        try:
            data = json.loads(msg.data)
            frame_count = data.get('count', 0)
            vision_ts   = data.get('timestamp', 0.0)
            frame_w     = float(data.get('frame_w', 640))
            frame_h     = float(data.get('frame_h', 480))

            # Abaikan jika menerima data dari frame yang sama
            if frame_count == self.last_frame_count:
                return
            self.last_frame_count = frame_count

            # Simpan metadata frame
            self.last_vision_timestamp = vision_ts
            self.last_vision_frame_no  = frame_count
            self.last_vision_frame_w   = frame_w
            self.last_vision_frame_h   = frame_h

            self.marker_detected = data.get('detected', False)

            if self.marker_detected and len(data.get('markers', [])) > 0:
                recv_time = self.get_current_time()
                self.last_detection_time = recv_time
                self.first_detection_made = True

                marker = data['markers'][0]
                center_x, center_y = marker['center']

                # Abaikan jika koordinat marker persis sama (frame duplikat)
                if center_x == self.last_center_x and center_y == self.last_center_y:
                    return
                self.last_center_x = center_x
                self.last_center_y = center_y
                self.last_raw_center_x = center_x
                self.last_raw_center_y = center_y

                # ── Parsal & Validasi Pose 3D (Persis sama dengan mission_debug_aruco) ──
                has_tvec = 'pose_tvec' in marker
                pose_valid = True
                tx, ty, tz = 0.0, 0.0, 0.0

                if has_tvec:
                    try:
                        tx, ty, tz = [float(v) for v in marker['pose_tvec']]
                        if tz <= 0.0 or tz > 10.0:
                            pose_valid = False
                        if abs(tx) > tz * 1.5 or abs(ty) > tz * 1.5:
                            pose_valid = False
                    except Exception:
                        pose_valid = False

                # Normalisasi error menggunakan dimensi aktual dari frame (menyerupai commit 51fe8a59)
                self.norm_error_x = (center_x - (frame_w / 2.0)) / (frame_w / 2.0)
                self.norm_error_y = (center_y - (frame_h / 2.0)) / (frame_h / 2.0)

                # ── Yaw dari pose_rvec ──
                if 'pose_rvec' in marker:
                    try:
                        # PENTING: rvec dari solvePnP adalah Rodrigues axis-angle, BUKAN Euler angle.
                        # Mengambil rvec[2] mentah menghasilkan nilai yang inkonsisten karena nilainya
                        # bergantung pada seluruh axis rotasi.
                        # Solusi benar: konversi ke rotation matrix lalu ambil yaw via atan2.
                        rvec_arr = np.array(marker['pose_rvec'], dtype=np.float64).reshape(3, 1)
                        R, _ = cv2.Rodrigues(rvec_arr)
                        self.yaw_error = math.atan2(R[1, 0], R[0, 0])  # range [-pi, pi]
                        self.has_yaw_measurement = True
                    except Exception:
                        pass

                # Log event deteksi ke CSV (src=VISION)
                if self.csv_file:
                    try:
                        dist = math.sqrt(self.norm_error_x**2 + self.norm_error_y**2)
                        alt  = self.get_current_altitude()
                        self.csv_file.write(
                            f"VISION,{recv_time:.4f},{self.loop_iter},{vision_ts:.4f},"
                            f"{frame_count},{int(frame_w)},{int(frame_h)},"
                            f"{center_x:.2f},{center_y:.2f},"
                            f"{self.norm_error_x:.5f},{self.norm_error_y:.5f},"
                            f"{dist:.4f},{alt:.3f},{self.yaw_error:.5f},VISION,"
                            f",,,,,,,,,,,,,,,,,,,,,,\n"
                        )
                        self.csv_file.flush()
                    except Exception:
                        pass
            else:
                self.marker_detected = False
                # Log frame tidak terdeteksi
                if self.csv_file:
                    try:
                        recv_time = self.get_current_time()
                        alt  = self.get_current_altitude()
                        self.csv_file.write(
                            f"VISION_NODET,{recv_time:.4f},{self.loop_iter},{vision_ts:.4f},"
                            f"{frame_count},{int(frame_w)},{int(frame_h)},"
                            f",,,,,{alt:.3f},,VISION_NODET,"
                            f",,,,,,,,,,,,,,,,,,,,,,\n"
                        )
                        self.csv_file.flush()
                    except Exception:
                        pass
        except Exception as e:
            self.write_warn(f"Gagal melakukan parsing data deteksi: {e}")

    def _compute_altitude_hold_rc3(self):
        """
        Hitung nilai RC3 (throttle) untuk active altitude hold menggunakan method terpusat VtolBaseNode.
        Tujuan: mempertahankan self.takeoff_altitude selama fase centering.
        """
        return self.compute_altitude_hold_rc3(target_altitude=self.takeoff_altitude)

    def run_pid_and_set_rc(self, current_time):
        """
        Dipanggil dari loop execute_centering dengan frekuensi konsisten (~20Hz).
        Ini satu-satunya tempat PID diupdate dan rc_channels ditulis.

        URUTAN FASE:
          1. YAW_ALIGN  — koreksi RC4 (yaw) sampai yaw_error < threshold
                          Roll/Pitch dibiarkan netral agar drone tidak bergeser saat rotate
          2. CENTERING  — koreksi RC1/RC2 (roll/pitch) menuju pusat marker
                          Yaw dipertahankan netral (aligned sudah)
        """
        distance_error = math.sqrt(self.norm_error_x**2 + self.norm_error_y**2)
        rc3_alt_hold   = self._compute_altitude_hold_rc3()

        # ── FASE 1: YAW ALIGN ──────────────────────────────────────────────
        if self.phase == 'YAW_ALIGN':
            yaw_abs = abs(self.yaw_error)
            # Transisi ke CENTERING hanya jika sudah ada pengukuran yaw valid DAN error di bawah threshold
            if self.has_yaw_measurement and yaw_abs <= self.yaw_error_threshold:
                # Yaw sudah aligned → transisi ke CENTERING
                if not self.yaw_aligned:
                    self.write_log(
                        f"[YAW] Aligned! yaw_err={self.yaw_error:.3f}rad <= "
                        f"threshold={self.yaw_error_threshold:.3f}rad. "
                        f"Mulai fase CENTERING."
                    )
                    self.yaw_aligned = True
                    self.pid_yaw.reset()
                    self.pid_roll.reset()
                    self.pid_pitch.reset()
                    self.phase = 'CENTERING'
                # Netral sambil transisi
                self.rc_channels[0] = 1500
                self.rc_channels[1] = 1500
                self.rc_channels[2] = rc3_alt_hold
                self.rc_channels[3] = 1500
                u_yaw_raw = 0.0
                u_yaw = 0.0
            else:
                # Koreksi yaw: RC4 saja dengan apply_smooth_deadzone, Roll/Pitch netral
                u_yaw_raw = self.pid_yaw.update(self.yaw_error, current_time)
                deadzone_bias_yaw = self.pid_params.get('deadzone_bias_yaw', 35.0)
                u_yaw_dz  = apply_smooth_deadzone(u_yaw_raw, deadzone_bias_yaw, band=0.5)
                max_yaw   = self.pid_params.get('max_yaw_override', 80)
                u_yaw     = max(min(u_yaw_dz, max_yaw), -max_yaw)
                self.rc_channels[0] = 1500          # Roll netral
                self.rc_channels[1] = 1500          # Pitch netral
                self.rc_channels[2] = rc3_alt_hold  # Altitude hold
                self.rc_channels[3] = int(1500 + u_yaw)
                self.get_logger().info(
                    f"[YAW_ALIGN] yaw_err={self.yaw_error:+.3f}rad u_yaw_raw={u_yaw_raw:+.1f} u_yaw={u_yaw:+.1f} RC4={self.rc_channels[3]}",
                    throttle_duration_sec=0.3
                )

            # Tulis CSV YAW_ALIGN
            if self.csv_file:
                try:
                    alt = self.get_current_altitude()
                    pid_dt_yaw = self.pid_yaw.last_dt
                    self.csv_file.write(
                        f"YAW,{current_time:.4f},{self.loop_iter},{self.last_vision_timestamp:.4f},"
                        f"{self.last_vision_frame_no},{int(self.last_vision_frame_w)},{int(self.last_vision_frame_h)},"
                        f"{self.last_raw_center_x:.2f},{self.last_raw_center_y:.2f},"
                        f"{self.norm_error_x:.5f},{self.norm_error_y:.5f},"
                        f"{distance_error:.4f},{alt:.3f},{self.yaw_error:.5f},YAW_ALIGN,"
                        f"{pid_dt_yaw:.4f},"
                        f",,,,,{self.rc_channels[0]},"
                        f",,,,,{self.rc_channels[1]},"
                        f"{self.pid_yaw.p_term:.4f},{self.pid_yaw.i_term:.4f},{self.pid_yaw.d_term:.4f},"
                        f"{u_yaw_raw:.4f},{u_yaw:.4f},{self.rc_channels[3]},"
                        f"{self.rc_channels[2]},0.000\n"
                    )
                    self.csv_file.flush()
                except Exception:
                    pass
            return  # Jangan lanjut ke centering dulu

        # ── FASE 2: CENTERING ──────────────────────────────────────────────
        # Check Heading Drift Re-entry Hysteresis (hanya jika enable_yaw_alignment aktif).
        # Threshold dikalikan 4.0 (bukan 2.0) agar tidak terlalu sensitif terhadap
        # noise yaw measurement yang menyebabkan bolak-balik YAW ↔ CENTERING.
        yaw_reenter_thresh = self.yaw_error_threshold * 4.0
        if self.enable_yaw_alignment and self.has_yaw_measurement and abs(self.yaw_error) > yaw_reenter_thresh:
            self.write_warn(
                f"[YAW_DRIFT] Heading miring signifikan! yaw_err={self.yaw_error:.3f}rad > "
                f"reenter_threshold={yaw_reenter_thresh:.3f}rad. Kembali ke YAW_ALIGN."
            )
            self.yaw_aligned = False
            self.phase = 'YAW_ALIGN'
            self.stable_start_time = None  # Reset stable timer saat re-enter YAW_ALIGN
            self.pid_yaw.reset()
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = rc3_alt_hold
            self.rc_channels[3] = 1500
            return

        u_roll_raw  = 0.0
        u_roll      = 0.0
        u_pitch_raw = 0.0
        u_pitch     = 0.0

        is_fresh = (current_time - self.last_detection_time) <= 0.3
        is_xy_centered = (distance_error <= self.error_threshold)
        # Catatan: is_yaw_aligned TIDAK digunakan sebagai syarat selesai di sini.
        # Yaw sudah di-align di fase YAW_ALIGN. Di CENTERING, yaw hanya dikoreksi
        # jika drift melebihi threshold (lihat yaw_active di bawah), tapi tidak
        # memblokir transisi ke kondisi stabil.
        is_yaw_ok = (not self.has_yaw_measurement) or (abs(self.yaw_error) <= self.yaw_error_threshold * 1.5)

        if is_fresh and is_xy_centered:
            # XY sudah di dalam toleransi: netralkan Roll/Pitch.
            # Yaw juga dinetralisir kecuali masih ada drift signifikan.
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = rc3_alt_hold
            self.rc_channels[3] = 1500
            self.pid_roll.reset()
            self.pid_pitch.reset()

            if self.stable_start_time is None:
                self.stable_start_time = current_time
                self.write_log(
                    f"Drone presisi di dalam toleransi XY (dist={distance_error:.3f}m "
                    f"<= {self.error_threshold}m). "
                    f"Menunggu stabil {self.centering_duration}s..."
                )
        else:
            if self.stable_start_time is not None:
                reasons = []
                if not is_fresh: reasons.append("vision stale >0.3s")
                if not is_xy_centered: reasons.append(f"dist={distance_error:.3f}m > {self.error_threshold}m")
                self.write_log(f"Drone keluar dari toleransi ({', '.join(reasons)}). Reset timer stabil...")
                self.stable_start_time = None

            # Update PID dari loop dengan dt yang konsisten
            u_roll_raw  = self.pid_roll.update(self.norm_error_x, current_time)
            u_pitch_raw = self.pid_pitch.update(self.norm_error_y, current_time)

            # Yaw PID dihitung hanya jika enable_yaw_alignment aktif, measurement ada DAN error melebihi threshold
            yaw_active = self.enable_yaw_alignment and self.has_yaw_measurement and (abs(self.yaw_error) > self.yaw_error_threshold)
            u_yaw_raw  = self.pid_yaw.update(self.yaw_error, current_time) if yaw_active else 0.0

            # Kompensasi deadzone RC dengan smooth transition
            deadzone_bias     = self.pid_params.get('deadzone_bias', 25.0)
            deadzone_bias_yaw = self.pid_params.get('deadzone_bias_yaw', 15.0)
            u_roll  = apply_smooth_deadzone(u_roll_raw,  deadzone_bias, band=0.5)
            u_pitch = apply_smooth_deadzone(u_pitch_raw, deadzone_bias, band=0.5)
            u_yaw   = apply_smooth_deadzone(u_yaw_raw,   deadzone_bias_yaw, band=0.5) if yaw_active else 0.0

            # Clamp ke max_override
            max_yaw = self.pid_params.get('max_yaw_override', 60)
            u_roll  = max(min(u_roll,  self.max_override), -self.max_override)
            u_pitch = max(min(u_pitch, self.max_override), -self.max_override)
            u_yaw   = max(min(u_yaw,   max_yaw), -max_yaw) if yaw_active else 0.0

            self.rc_channels[0] = int(1500 + u_roll)
            self.rc_channels[1] = int(1500 + u_pitch)
            self.rc_channels[2] = rc3_alt_hold  # Active altitude hold
            self.rc_channels[3] = int(1500 + u_yaw)  # Active Yaw hold jika miring > threshold

        # Log diagnostik ke file teks per iterasi
        log_str = (
            f"DEBUG PID:\n"
            f"  Roll : Err={self.norm_error_x:.3f} | Raw={u_roll_raw:.2f} "
            f"(P={self.pid_roll.p_term:.2f}, I={self.pid_roll.i_term:.2f}, D={self.pid_roll.d_term:.2f}) "
            f"| Final={u_roll:.2f} -> RC={self.rc_channels[0]}\n"
            f"  Pitch: Err={self.norm_error_y:.3f} | Raw={u_pitch_raw:.2f} "
            f"(P={self.pid_pitch.p_term:.2f}, I={self.pid_pitch.i_term:.2f}, D={self.pid_pitch.d_term:.2f}) "
            f"| Final={u_pitch:.2f} -> RC={self.rc_channels[1]}\n"
            f"  Dist={distance_error:.2f} | Alt={self.get_current_altitude():.2f}m"
        )
        if self.log_file:
            try:
                self.log_file.write(f"[{time.strftime('%H:%M:%S')}] {log_str}\n")
                self.log_file.flush()
            except Exception:
                pass
        self.get_logger().info(log_str, throttle_duration_sec=0.5)

        # Log baris CSV terstruktur per iterasi PID loop
        if self.csv_file:
            try:
                alt = self.get_current_altitude()
                pid_dt = self.pid_roll.last_dt
                stable_dur = (current_time - self.stable_start_time) if self.stable_start_time else 0.0
                self.csv_file.write(
                    f"PID,{current_time:.4f},{self.loop_iter},{self.last_vision_timestamp:.4f},"
                    f"{self.last_vision_frame_no},{int(self.last_vision_frame_w)},{int(self.last_vision_frame_h)},"
                    f"{self.last_raw_center_x:.2f},{self.last_raw_center_y:.2f},"
                    f"{self.norm_error_x:.5f},{self.norm_error_y:.5f},"
                    f"{distance_error:.4f},{alt:.3f},{self.yaw_error:.5f},CENTERING,"
                    f"{pid_dt:.4f},"
                    f"{self.pid_roll.p_term:.4f},{self.pid_roll.i_term:.4f},{self.pid_roll.d_term:.4f},"
                    f"{u_roll_raw:.4f},{u_roll:.4f},{self.rc_channels[0]},"
                    f"{self.pid_pitch.p_term:.4f},{self.pid_pitch.i_term:.4f},{self.pid_pitch.d_term:.4f},"
                    f"{u_pitch_raw:.4f},{u_pitch:.4f},{self.rc_channels[1]},"
                    f"{self.pid_yaw.p_term:.4f},{self.pid_yaw.i_term:.4f},{self.pid_yaw.d_term:.4f},"
                    f"{u_yaw_raw:.4f},{u_yaw:.4f},{self.rc_channels[3]},"
                    f"{self.rc_channels[2]},{stable_dur:.3f}\n"
                )
                self.csv_file.flush()
            except Exception:
                pass

    def _stabilize_with_altitude_hold(self, duration_seconds):
        """
        Hover selama duration_seconds dengan active altitude hold.
        Memanggil method hover() terpusat di VtolBaseNode.
        """
        return self.hover(duration_seconds, target_altitude=self.takeoff_altitude)

    def execute_centering(self):
        self.write_log("Menunggu marker ArUco terdeteksi...")

        # Takeoff ke ketinggian yang terkonfigurasi
        if not self.takeoff(target_altitude=self.takeoff_altitude):
            return

        # Stabilisasi awal dengan altitude hold aktif.
        # TIDAK menggunakan base class hover() karena RC3=1500 hardcoded di LOITER
        # menyebabkan drone turun (deadband/THR_MID bias di SITL).
        if not self._stabilize_with_altitude_hold(duration_seconds=3.0):
            return

        self.write_log("Mulai fase Centering presisi...")
        self.pid_roll.reset()
        self.pid_pitch.reset()

        centering_start_time = self.get_current_time()
        self.centering_active = True

        rate = self.create_rate(20.0) # Rate loop stabil 20Hz
        while rclpy.ok() and not self.centered:
            current_time = self.get_current_time()
            self.loop_iter += 1

            # Watchdog: Proteksi keselamatan mode terbang manual
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.write_warn(
                    f"Intervensi manual terdeteksi! Mode berubah ke "
                    f"{self.current_state.mode}. Abort misi."
                )
                self.abort_flight()
                return

            # Safety watchdog: Kehilangan sinyal marker setelah terdeteksi setidaknya sekali
            if self.first_detection_made:
                if current_time - self.last_detection_time > self.marker_lost_timeout:
                    self.write_warn(
                        f"Marker hilang selama lebih dari {self.marker_lost_timeout} detik! "
                        f"Menghentikan pergerakan & memicu failsafe LAND..."
                    )
                    self.rc_channels[0] = 1500
                    self.rc_channels[1] = 1500
                    self.rc_channels[2] = 1500  # Netral throttle -> LOITER maintain altitude
                    self.rc_channels[3] = 1500
                    self.publish_rc()
                    self.land()
                    return
            else:
                if current_time - centering_start_time > 15.0:
                    self.write_warn(
                        "Marker tidak terdeteksi setelah 15 detik awal. "
                        "Menggagalkan misi & landing..."
                    )
                    self.land()
                    return

            # Jalankan PID dan update rc_channels hanya jika marker terdeteksi
            if self.marker_detected and self.centering_active:
                self.run_pid_and_set_rc(current_time)
            elif self.centering_active:
                # Marker sementara hilang: hover di tempat tanpa mereset PID state,
                # tapi tetap aktifkan altitude hold agar tidak drift turun
                self.rc_channels[0] = 1500
                self.rc_channels[1] = 1500
                self.rc_channels[2] = self._compute_altitude_hold_rc3()  # Active altitude hold
                self.rc_channels[3] = 1500
                self.stable_start_time = None

            # Check if stabilized for centering_duration (wajib data vision fresh <= 0.3s)
            is_data_fresh = (current_time - self.last_detection_time) <= 0.3
            if not is_data_fresh and self.stable_start_time is not None:
                self.write_warn("Data vision stale saat penantian stabil! Reset timer stabil.")
                self.stable_start_time = None

            if (self.stable_start_time is not None
                    and current_time - self.stable_start_time >= self.centering_duration):
                # Pastikan saat stable timer habis kita memang masih di fase CENTERING
                if self.phase != 'CENTERING':
                    self.write_warn(
                        f"[STABLE] Stable timer habis tapi phase={self.phase}, bukan CENTERING! Reset timer."
                    )
                    self.stable_start_time = None
                else:
                    self.write_log(
                        f"Target stabil (XY centered, phase=CENTERING, Fresh Data) "
                        f"selama {self.centering_duration}s tercapai. Sukses!"
                    )
                    self.centered = True
                    break

            # Log altitude real-time setiap iterasi (20Hz) untuk diagnosa
            current_alt = self.get_current_altitude()
            alt_err = self.takeoff_altitude - current_alt
            self.get_logger().info(
                f"[ALT] {current_alt:.3f}m / target={self.takeoff_altitude:.1f}m "
                f"(err={alt_err:+.3f}m) | RC3={self.rc_channels[2]} | "
                f"marker={'YES' if self.marker_detected else 'NO'} | "
                f"err_xy=({self.norm_error_x:.3f},{self.norm_error_y:.3f})"
            )

            # Pengganti rate.sleep() untuk menghindari deadlock di ROS 2 (sim_time)
            # Selalu gunakan rclpy.spin_once() agar callback dan /clock tetap terproses
            target_dt = 0.05  # 20 Hz
            elapsed_in_loop = self.get_current_time() - current_time
            sleep_time = target_dt - elapsed_in_loop

            if sleep_time > 0.0:
                spin_start = self.get_current_time()
                while rclpy.ok() and (self.get_current_time() - spin_start) < sleep_time:
                    rclpy.spin_once(self, timeout_sec=0.01)
            else:
                rclpy.spin_once(self, timeout_sec=0.0)

        self.centering_active = False

        # Pendaratan otonom di atas marker
        if self.centered:
            self.write_log("Memulai pendaratan otomatis otonom tepat di atas marker...")
            self.land()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionCenteringNode()
    try:
        node.execute_centering()
    except KeyboardInterrupt:
        node.safe_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
