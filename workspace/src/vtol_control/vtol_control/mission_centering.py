import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_pid_config
import time
import json
import math

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

    def update(self, error, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            self.last_output = 0.0
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return self.last_output

        # Abaikan update jika dt terlalu kecil (kurang dari 10ms)
        if dt < 0.01:
            return self.last_output

        # Proportional
        self.p_term = self.kp * error

        # Integral dengan Anti-windup
        self.integral += error * dt
        max_integral = self.max_out / (self.ki if self.ki != 0 else 1.0)
        self.integral = max(min(self.integral, max_integral), -max_integral)
        self.i_term = self.ki * self.integral

        # Derivative dengan low-pass filter untuk meredam spike dari jitter dt
        raw_derivative = (error - self.last_error) / dt
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
        self.takeoff_altitude = self.pid_params['takeoff_altitude']

        # Inisialisasi PID dengan low-pass filter pada D-term (alpha=0.3)
        # max_out dikurangi deadzone_bias agar output final tidak melebihi max_override
        deadzone_bias = 10.0
        pid_max_raw = self.max_override - deadzone_bias
        self.pid_roll = PIDController(
            self.pid_params['kp_roll'], self.pid_params['ki_roll'],
            self.pid_params['kd_roll'], pid_max_raw, d_filter_alpha=0.3
        )
        self.pid_pitch = PIDController(
            self.pid_params['kp_pitch'], self.pid_params['ki_pitch'],
            self.pid_params['kd_pitch'], pid_max_raw, d_filter_alpha=0.3
        )

        self.write_log(
            f"LOADED PARAMETERS: kp_roll={self.pid_roll.kp}, ki_roll={self.pid_roll.ki}, "
            f"kd_roll={self.pid_roll.kd}, max_override={self.max_override}, "
            f"d_filter_alpha={self.pid_roll.d_filter_alpha}"
        )

        # Buka CSV log terstruktur untuk analisis post-flight
        try:
            self.csv_file = open(CSV_LOG_PATH, "w")
            self.csv_file.write(
                "src,wall_time,loop_iter,vision_ts,frame_no,frame_w,frame_h,"
                "center_x,center_y,norm_ex,norm_ey,dist,alt,"
                "pid_dt,p_roll,i_roll,d_roll,raw_roll,final_roll,rc_roll,"
                "p_pitch,i_pitch,d_pitch,raw_pitch,final_pitch,rc_pitch,"
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
        self.last_raw_center_x = 0.0
        self.last_raw_center_y = 0.0
        self.last_frame_count = -1
        self.last_center_x = -1.0
        self.last_center_y = -1.0
        self.loop_iter = 0

        self.stable_start_time = None
        self.centered = False
        self.centering_active = False

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
                recv_time = time.time()
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

                # Normalisasi error menggunakan dimensi aktual dari frame
                self.norm_error_x = (center_x - (frame_w / 2.0)) / (frame_w / 2.0)
                self.norm_error_y = (center_y - (frame_h / 2.0)) / (frame_h / 2.0)

                # Log event deteksi ke CSV (src=VISION)
                if self.csv_file:
                    try:
                        dist = math.sqrt(self.norm_error_x**2 + self.norm_error_y**2)
                        alt  = self.current_pose.pose.position.z
                        self.csv_file.write(
                            f"VISION,{recv_time:.4f},{self.loop_iter},{vision_ts:.4f},"
                            f"{frame_count},{int(frame_w)},{int(frame_h)},"
                            f"{center_x:.2f},{center_y:.2f},"
                            f"{self.norm_error_x:.5f},{self.norm_error_y:.5f},"
                            f"{dist:.4f},{alt:.3f},"
                            f",,,,,,,,,,,,\n"
                        )
                        self.csv_file.flush()
                    except Exception:
                        pass
            else:
                self.marker_detected = False
                # Log frame tidak terdeteksi
                if self.csv_file:
                    try:
                        recv_time = time.time()
                        self.csv_file.write(
                            f"VISION_NODET,{recv_time:.4f},{self.loop_iter},{vision_ts:.4f},"
                            f"{frame_count},{int(frame_w)},{int(frame_h)},"
                            f",,,,,{self.current_pose.pose.position.z:.3f},"
                            f",,,,,,,,,,,,\n"
                        )
                        self.csv_file.flush()
                    except Exception:
                        pass
        except Exception as e:
            self.write_warn(f"Gagal melakukan parsing data deteksi: {e}")

    def run_pid_and_set_rc(self, current_time):
        """
        Dipanggil dari loop execute_centering dengan frekuensi konsisten (~20Hz).
        Ini satu-satunya tempat PID diupdate dan rc_channels ditulis.
        """
        distance_error = math.sqrt(self.norm_error_x**2 + self.norm_error_y**2)

        if distance_error <= self.error_threshold:
            # Di dalam toleransi: netralkan Roll/Pitch, lepas throttle ke LOITER
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = 1500  # Netral throttle -> LOITER maintain altitude
            self.rc_channels[3] = 1500
            self.pid_roll.reset()
            self.pid_pitch.reset()

            u_roll_raw = 0.0
            u_roll = 0.0
            u_pitch_raw = 0.0
            u_pitch = 0.0

            if self.stable_start_time is None:
                self.stable_start_time = current_time
                self.write_log("Drone berada di dalam batas toleransi. Menunggu stabil...")
        else:
            if self.stable_start_time is not None:
                self.write_log("Drone keluar dari batas toleransi. Mengulang waktu tunggu...")
                self.stable_start_time = None

            # Update PID dari loop dengan dt yang konsisten
            u_roll_raw = self.pid_roll.update(self.norm_error_x, current_time)
            u_pitch_raw = self.pid_pitch.update(self.norm_error_y, current_time)

            # Kompensasi deadzone RC dengan smooth transition
            deadzone_bias = 10.0
            u_roll = apply_smooth_deadzone(u_roll_raw, deadzone_bias)
            u_pitch = apply_smooth_deadzone(u_pitch_raw, deadzone_bias)

            # Clamp ke max_override
            u_roll = max(min(u_roll, self.max_override), -self.max_override)
            u_pitch = max(min(u_pitch, self.max_override), -self.max_override)

            self.rc_channels[0] = int(1500 + u_roll)
            self.rc_channels[1] = int(1500 + u_pitch)
            self.rc_channels[2] = 1500  # Netral throttle -> LOITER maintain altitude
            self.rc_channels[3] = 1500

        # Log diagnostik ke file teks per iterasi
        log_str = (
            f"DEBUG PID:\n"
            f"  Roll : Err={self.norm_error_x:.3f} | Raw={u_roll_raw:.2f} "
            f"(P={self.pid_roll.p_term:.2f}, I={self.pid_roll.i_term:.2f}, D={self.pid_roll.d_term:.2f}) "
            f"| Final={u_roll:.2f} -> RC={self.rc_channels[0]}\n"
            f"  Pitch: Err={self.norm_error_y:.3f} | Raw={u_pitch_raw:.2f} "
            f"(P={self.pid_pitch.p_term:.2f}, I={self.pid_pitch.i_term:.2f}, D={self.pid_pitch.d_term:.2f}) "
            f"| Final={u_pitch:.2f} -> RC={self.rc_channels[1]}\n"
            f"  Dist={distance_error:.2f} | Alt={self.current_pose.pose.position.z:.2f}m"
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
                alt = self.current_pose.pose.position.z
                pid_dt = (current_time - self.pid_roll.last_time) if self.pid_roll.last_time else 0.0
                stable_dur = (current_time - self.stable_start_time) if self.stable_start_time else 0.0
                self.csv_file.write(
                    f"PID,{current_time:.4f},{self.loop_iter},{self.last_vision_timestamp:.4f},"
                    f"{self.last_vision_frame_no},{int(self.last_vision_frame_w)},{int(self.last_vision_frame_h)},"
                    f"{self.last_raw_center_x:.2f},{self.last_raw_center_y:.2f},"
                    f"{self.norm_error_x:.5f},{self.norm_error_y:.5f},"
                    f"{distance_error:.4f},{alt:.3f},"
                    f"{pid_dt:.4f},"
                    f"{self.pid_roll.p_term:.4f},{self.pid_roll.i_term:.4f},{self.pid_roll.d_term:.4f},"
                    f"{u_roll_raw:.4f},{u_roll:.4f},{self.rc_channels[0]},"
                    f"{self.pid_pitch.p_term:.4f},{self.pid_pitch.i_term:.4f},{self.pid_pitch.d_term:.4f},"
                    f"{u_pitch_raw:.4f},{u_pitch:.4f},{self.rc_channels[1]},"
                    f"{self.rc_channels[2]},{stable_dur:.3f}\n"
                )
                self.csv_file.flush()
            except Exception:
                pass

    def execute_centering(self):
        self.write_log("Menunggu marker ArUco terdeteksi...")

        # Takeoff ke ketinggian yang terkonfigurasi
        if not self.takeoff(target_altitude=self.takeoff_altitude):
            return

        # Hover sebentar untuk stabilisasi awal
        if not self.hover(duration_seconds=3.0):
            return

        self.write_log("Mulai fase Centering presisi...")
        self.pid_roll.reset()
        self.pid_pitch.reset()

        centering_start_time = time.time()
        self.centering_active = True

        while rclpy.ok() and not self.centered:
            current_time = time.time()
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
                # Marker sementara hilang: hover di tempat tanpa mereset PID state
                self.rc_channels[0] = 1500
                self.rc_channels[1] = 1500
                self.rc_channels[2] = 1500  # Netral throttle -> LOITER maintain altitude
                self.rc_channels[3] = 1500
                self.stable_start_time = None

            # Check if stabilized for centering_duration
            if (self.stable_start_time is not None
                    and current_time - self.stable_start_time >= self.centering_duration):
                self.write_log(
                    f"Target stabil selama {self.centering_duration}s tercapai. Sukses!"
                )
                self.centered = True
                break

            rclpy.spin_once(self, timeout_sec=0.05)

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
