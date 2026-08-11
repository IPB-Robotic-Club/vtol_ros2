import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_search_marker_config, get_takeoff_config
import time
import json


class MissionSearchMarkerNode(VtolBaseNode):
    """
    Mission node untuk melakukan Takeoff, pencarian ArUco Marker ID 2 dengan
    gerakan Roll Kanan bertahap (pulse + 1 detik delay), mengeksekusi Overshoot
    saat marker terdeteksi, lalu melakukan Hover dan Landing otonom.
    """
    def __init__(self):
        # Aktifkan background 10Hz RC publishing timer untuk override
        super().__init__('mission_search_marker_node', enable_rc_loop=True)

        # Membaca konfigurasi dari vtol_config.yaml
        sm_config = get_search_marker_config()
        self.target_marker_id = sm_config.get('target_marker_id', 2)
        self.roll_override = sm_config.get('roll_override', 60)
        self.pulse_duration = sm_config.get('pulse_duration', 0.5)
        self.pause_duration = sm_config.get('pause_duration', 1.0)
        self.overshoot_pulse_duration = sm_config.get('overshoot_pulse_duration', 0.5)
        self.hover_duration_before = sm_config.get('hover_duration_before', 2.0)
        self.hover_duration_after = sm_config.get('hover_duration_after', 3.0)
        self.alt_correction_threshold = sm_config.get('alt_correction_threshold', 0.9)
        self.tilt_compensation_gain = sm_config.get('tilt_compensation_gain', 0.25)
        self.post_detection_pulses = sm_config.get('post_detection_pulses', 2)
        self.post_detection_direction = sm_config.get('post_detection_direction', 'left')
        self.post_detection_roll_override = sm_config.get('post_detection_roll_override', -60)
        self.post_detection_pitch_override = sm_config.get('post_detection_pitch_override', 0)
        self.post_detection_pulse_duration = sm_config.get('post_detection_pulse_duration', 0.5)
        self.post_detection_pause_duration = sm_config.get('post_detection_pause_duration', 1.0)

        # State tracking deteksi marker
        self.marker_detected = False
        self.last_detection_time = 0.0
        self.detected_marker_info = None

        # Subscription ke vision output ArUco
        self.aruco_sub = self.create_subscription(
            String,
            '/vtol/aruco/detection',
            self._aruco_callback,
            self.qos_telemetry
        )

        self.get_logger().info(
            f"[MissionSearchMarker] Node Siap! "
            f"Target ID={self.target_marker_id}, Roll Override=+{self.roll_override} PWM ({1500+self.roll_override}), "
            f"Pulse={self.pulse_duration}s, Pause={self.pause_duration}s, "
            f"Overshoot={self.overshoot_pulse_duration}s, "
            f"PostDetPulses={self.post_detection_pulses}x ({self.post_detection_direction}), "
            f"AltThresh={self.alt_correction_threshold}m, TiltGain={self.tilt_compensation_gain}"
        )

    def _aruco_callback(self, msg: String):
        """Callback untuk memproses data deteksi ArUco dari vtol_vision."""
        try:
            data = json.loads(msg.data)
            if not data.get('detected', False):
                return

            markers = data.get('markers', [])
            for m in markers:
                if m.get('id') == self.target_marker_id:
                    self.marker_detected = True
                    self.last_detection_time = time.time()
                    self.detected_marker_info = m
                    break
        except Exception as e:
            self.get_logger().warn(f"Gagal memparse data ArUco: {e}")

    def execute_roll_right_search(self, target_altitude=None) -> bool:
        """
        Menjalankan loop dorongan roll kanan bertahap (pulse + 1s delay)
        hingga marker target terdeteksi, lalu mengeksekusi overshoot.
        Saat fase jeda 1s (diam), sistem secara aktif memantau & mengoreksi ketinggian.
        """
        if target_altitude is None:
            from vtol_control.config_reader import get_takeoff_config
            target_altitude = get_takeoff_config().get('takeoff_altitude', 1.2)

        self.get_logger().info(
            f"[SEARCH] Memulai siklus roll kanan bertahap hingga marker ID {self.target_marker_id} terdeteksi... "
            f"(Target Alt: {target_altitude:.2f}m)"
        )

        step_counter = 0

        while rclpy.ok():
            # Watchdog check: koneksi MAVROS
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected saat pencarian! Aborting.")
                self.abort_flight()
                return False

            # Watchdog check: manual override
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(
                    f"Manual override terdeteksi! Mode penerbangan berubah ke {self.current_state.mode}. Aborting mission."
                )
                self.abort_flight()
                return False

            # Cek apakah marker sudah terdeteksi
            if self.marker_detected:
                break

            step_counter += 1
            self.get_logger().info(
                f"[SEARCH Step {step_counter}] Dorong Roll Kanan (PWM: {1500 + self.roll_override}) selama {self.pulse_duration}s..."
            )

            # ── Phase 1: Roll Right Pulse (dengan Tilt Feedforward Compensation) ──
            start_pulse = time.time()
            while rclpy.ok() and (time.time() - start_pulse < self.pulse_duration):
                if self.marker_detected:
                    break

                self.rc_channels[0] = 1500 + self.roll_override  # Roll kanan
                self.rc_channels[1] = 1500                      # Pitch netral
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3                       # Throttle alt hold + tilt boost
                self.rc_channels[3] = 1500                      # Yaw netral

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

            if self.marker_detected:
                break

            # ── Phase 2: Pause / Delay Interval + Active Altitude Correction (Saat Diam) ──
            self.rc_channels[0] = 1500  # Roll netral saat diam
            self.rc_channels[1] = 1500  # Pitch netral
            self.rc_channels[3] = 1500  # Yaw netral

            start_pause = time.time()
            corrected_log_shown = False

            while rclpy.ok() and (time.time() - start_pause < self.pause_duration):
                if self.marker_detected:
                    break

                current_alt = self.get_current_altitude()
                alt_error = target_altitude - current_alt
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3

                if not self.rc_timer:
                    self.publish_rc()

                # Cetak log koreksi jika drone turun melampaui batas toleransi
                if alt_error > self.alt_correction_threshold and not corrected_log_shown:
                    self.get_logger().info(
                        f"[ALT CORRECTION] Drone drop -{alt_error:.3f}m di bawah target ({current_alt:.2f}m/{target_altitude:.2f}m) -> "
                        f"Mengoreksi RC3 ke {rc3} PWM...",
                        throttle_duration_sec=0.5
                    )
                    corrected_log_shown = True

                rclpy.spin_once(self, timeout_sec=0.05)

            # Jika setelah 1s pause ketinggian masih drop, perpanjang koreksi sejenak hingga pulih (< 0.03m)
            recovery_start = time.time()
            while rclpy.ok() and (target_altitude - self.get_current_altitude() > 0.03) and (time.time() - recovery_start < 1.0):
                if self.marker_detected:
                    break
                current_alt = self.get_current_altitude()
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3
                if not self.rc_timer:
                    self.publish_rc()
                self.get_logger().info(
                    f"[ALT RECOVERY] Memulihkan ketinggian... {current_alt:.2f}m -> {target_altitude:.2f}m (RC3: {rc3})",
                    throttle_duration_sec=0.3
                )
                rclpy.spin_once(self, timeout_sec=0.05)

        # Check jika keluar loop karena marker terdeteksi
        if self.marker_detected:
            self.get_logger().info(
                f"\n=======================================================\n"
                f" >>> ARUCO MARKER ID {self.target_marker_id} TERDETEKSI! <<<\n"
                f"=======================================================\n"
            )

            # ── Phase 3: Overshoot Execution ───────────────────────────────
            self.get_logger().info(
                f"[OVERSHOOT] Mengeksekusi dorongan overshoot roll kanan selama {self.overshoot_pulse_duration}s..."
            )
            start_overshoot = time.time()
            while rclpy.ok() and (time.time() - start_overshoot < self.overshoot_pulse_duration):
                rc3 = self.compute_altitude_hold_rc3()
                self.rc_channels[0] = 1500 + self.roll_override
                self.rc_channels[1] = 1500
                self.rc_channels[2] = rc3
                self.rc_channels[3] = 1500

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

            # Netralkan kontrol pasca-overshoot
            self.get_logger().info("[OVERSHOOT] Overshoot selesai. Menetralkan kontrol Roll & Pitch.")
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[3] = 1500
            if not self.rc_timer:
                self.publish_rc()

            return True

        return False

    def execute_post_detection_maneuver(self, target_altitude=None) -> bool:
        """
        Menjalankan dorongan maneuver pasca-deteksi ArUco 2 (misalnya 2 kali dorongan ke kiri)
        setelah marker terdeteksi & overshoot selesai.
        """
        if self.post_detection_pulses <= 0:
            return True

        if target_altitude is None:
            from vtol_control.config_reader import get_takeoff_config
            target_altitude = get_takeoff_config().get('takeoff_altitude', 1.2)

        self.get_logger().info(
            f"[POST-DETECTION] Mengeksekusi {self.post_detection_pulses} kali dorongan '{self.post_detection_direction}' "
            f"(Roll PWM: {1500 + self.post_detection_roll_override}, Pitch PWM: {1500 + self.post_detection_pitch_override})..."
        )

        for step in range(1, self.post_detection_pulses + 1):
            if not rclpy.ok():
                return False

            self.get_logger().info(
                f"[POST-DETECTION Step {step}/{self.post_detection_pulses}] Dorong {self.post_detection_direction} selama {self.post_detection_pulse_duration}s..."
            )

            # Phase 1: Dorongan Pulsa Pasca-Deteksi (dengan Active Alt Hold)
            start_pulse = time.time()
            while rclpy.ok() and (time.time() - start_pulse < self.post_detection_pulse_duration):
                self.rc_channels[0] = 1500 + self.post_detection_roll_override
                self.rc_channels[1] = 1500 + self.post_detection_pitch_override
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3
                self.rc_channels[3] = 1500

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

            # Phase 2: Pause / Delay netral antar pulsa
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[3] = 1500

            start_pause = time.time()
            while rclpy.ok() and (time.time() - start_pause < self.post_detection_pause_duration):
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("[POST-DETECTION] Dorongan pasca-deteksi selesai. Menetralkan kontrol Roll & Pitch.")
        self.rc_channels[0] = 1500
        self.rc_channels[1] = 1500
        self.rc_channels[3] = 1500
        if not self.rc_timer:
            self.publish_rc()

        return True

    def run_mission(self):
        """Menjalankan seluruh alur misi otonom."""
        # 1. Takeoff otonom ke target altitude
        if not self.takeoff():
            self.get_logger().error("Misi dibatalkan: Takeoff gagal.")
            return

        # 2. Hover stabilisasi awal
        if not self.hover(duration_seconds=self.hover_duration_before):
            self.get_logger().error("Misi dibatalkan: Hover awal gagal.")
            return

        # 3. Pencarian Roll Kanan + Deteksi Marker ID 2 + Overshoot
        if not self.execute_roll_right_search():
            self.get_logger().error("Misi dibatalkan: Pencarian marker terinterupsi.")
            return

        # 4. Dorongan pasca-deteksi (dorong ke kiri 2 kali)
        if not self.execute_post_detection_maneuver():
            self.get_logger().error("Misi dibatalkan: Maneuver pasca-deteksi terinterupsi.")
            return

        # 5. Hover pasca-maneuver untuk stabilisasi
        if not self.hover(duration_seconds=self.hover_duration_after):
            self.get_logger().error("Misi dibatalkan: Hover akhir gagal.")
            return

        # 6. Pendaratan otonom dan cleanup
        self.get_logger().info("Misi pencarian, overshoot & maneuver pasca-deteksi selesai! Melakukan pendaratan...")
        self.land()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionSearchMarkerNode()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.safe_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
