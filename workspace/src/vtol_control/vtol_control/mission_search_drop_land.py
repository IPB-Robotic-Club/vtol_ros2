import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_search_marker_config, get_takeoff_config
import time
import json


class MissionSearchDropLandNode(VtolBaseNode):
    """
    Misi 11: Mission node untuk melakukan Takeoff, pencarian ArUco Marker ID 2
    dengan gerakan Roll Kanan bertahap, Overshoot, Dorongan Pasca-Deteksi ke Kiri (2x),
    Drop Payload via Servo (RC CH9 PWM 1900 -> 1100), dan Landing otonom.
    """
    def __init__(self):
        # Aktifkan background 10Hz RC publishing timer untuk override
        super().__init__('mission_search_drop_land_node', enable_rc_loop=True)

        # Membaca konfigurasi dari vtol_config.yaml
        sm_config = get_search_marker_config()
        self.target_marker_id = sm_config.get('target_marker_id', 2)
        self.roll_override = sm_config.get('roll_override', 60)
        self.pulse_duration = sm_config.get('pulse_duration', 0.5)
        self.pause_duration = sm_config.get('pause_duration', 1.5)
        self.overshoot_pulse_duration = sm_config.get('overshoot_pulse_duration', 0.5)
        self.hover_duration_before = sm_config.get('hover_duration_before', 2.0)
        self.hover_duration_after = sm_config.get('hover_duration_after', 3.0)
        self.alt_correction_threshold = sm_config.get('alt_correction_threshold', 0.9)
        self.tilt_compensation_gain = sm_config.get('tilt_compensation_gain', 0.25)

        # Parameter maneuver pasca-deteksi (sebelum drop)
        self.post_detection_pulses = sm_config.get('post_detection_pulses', 2)
        self.post_detection_direction = sm_config.get('post_detection_direction', 'left')
        self.post_detection_roll_override = sm_config.get('post_detection_roll_override', -60)
        self.post_detection_pitch_override = sm_config.get('post_detection_pitch_override', 0)
        self.post_detection_pulse_duration = sm_config.get('post_detection_pulse_duration', 0.5)
        self.post_detection_pause_duration = sm_config.get('post_detection_pause_duration', 1.5)

        # Parameter Servo Drop
        self.servo_channel = sm_config.get('servo_channel', 9)
        self.servo_initial_pwm = sm_config.get('servo_initial_pwm', 1900)
        self.servo_drop_pwm = sm_config.get('servo_drop_pwm', 1100)
        self.servo_drop_duration = sm_config.get('servo_drop_duration', 2.0)

        # Parameter maneuver pasca-drop (setelah drop & sebelum landing)
        self.post_drop_pulses = sm_config.get('post_drop_pulses', 2)
        self.post_drop_direction = sm_config.get('post_drop_direction', 'left')
        self.post_drop_roll_override = sm_config.get('post_drop_roll_override', -60)
        self.post_drop_pitch_override = sm_config.get('post_drop_pitch_override', 0)
        self.post_drop_pulse_duration = sm_config.get('post_drop_pulse_duration', 0.5)
        self.post_drop_pause_duration = sm_config.get('post_drop_pause_duration', 1.5)

        # Inisialisasi posisi awal servo ke locked (misal: 1900 PWM)
        self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

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
            f"[MissionSearchDropLand] Node Siap! "
            f"Target ID={self.target_marker_id}, Servo CH{self.servo_channel} Init PWM={self.servo_initial_pwm}, "
            f"Drop PWM={self.servo_drop_pwm}, Roll Override=+{self.roll_override} PWM, "
            f"PostDetPulses={self.post_detection_pulses}x ({self.post_detection_direction})"
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
        Menjalankan loop dorongan roll kanan bertahap (pulse + delay)
        hingga marker target terdeteksi, lalu mengeksekusi overshoot.
        """
        if target_altitude is None:
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

            # ── Phase 1: Roll Right Pulse ──────────────────────────────────
            start_pulse = time.time()
            while rclpy.ok() and (time.time() - start_pulse < self.pulse_duration):
                if self.marker_detected:
                    break

                self.rc_channels[0] = 1500 + self.roll_override  # Roll kanan
                self.rc_channels[1] = 1500                      # Pitch netral
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3                       # Throttle alt hold + tilt boost
                self.rc_channels[3] = 1500                      # Yaw netral
                # Pastikan servo CH9 tetap di 1900 PWM saat pencarian
                self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

            if self.marker_detected:
                break

            # ── Phase 2: Pause / Delay Interval + Active Altitude Correction ──
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
                self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

                if not self.rc_timer:
                    self.publish_rc()

                if alt_error > self.alt_correction_threshold and not corrected_log_shown:
                    self.get_logger().info(
                        f"[ALT CORRECTION] Drone drop -{alt_error:.3f}m di bawah target ({current_alt:.2f}m/{target_altitude:.2f}m) -> "
                        f"Mengoreksi RC3 ke {rc3} PWM...",
                        throttle_duration_sec=0.5
                    )
                    corrected_log_shown = True

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
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[0] = 1500 + self.roll_override
                self.rc_channels[1] = 1500
                self.rc_channels[2] = rc3
                self.rc_channels[3] = 1500
                self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

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
                self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

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
                self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

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

    def drop_payload(self, target_altitude=None) -> bool:
        """
        Mengeksekusi pelepas beban (Servo Drop):
        Mengubah PWM Servo dari servo_initial_pwm (1900) ke servo_drop_pwm (1100),
        lalu menahan posisi (hover) selama servo_drop_duration detik sebelum landing.
        """
        if target_altitude is None:
            target_altitude = get_takeoff_config().get('takeoff_altitude', 1.2)

        self.get_logger().info(
            f"\n=======================================================\n"
            f" >>> MELEPAS PAYLOAD VIA SERVO CH{self.servo_channel} (PWM: {self.servo_initial_pwm} -> {self.servo_drop_pwm}) <<<\n"
            f"=======================================================\n"
        )

        # Ubah PWM servo ke nilai drop (1100 PWM)
        self.set_rc_channel(self.servo_channel, self.servo_drop_pwm)

        # Tahan posisi di udara dengan altitude hold selama servo_drop_duration
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < self.servo_drop_duration):
            rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = rc3
            self.rc_channels[3] = 1500
            self.set_rc_channel(self.servo_channel, self.servo_drop_pwm)

            if not self.rc_timer:
                self.publish_rc()

            current_alt = self.get_current_altitude()
            self.get_logger().info(
                f"[SERVO DROP] Holding altitude: {current_alt:.2f}m/{target_altitude:.2f}m | "
                f"CH{self.servo_channel} PWM: {self.servo_drop_pwm} | elapsed: {time.time() - start_time:.1f}s",
                throttle_duration_sec=0.5
            )
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"[SERVO DROP] Payload selesai dilepas! (Ditahan {self.servo_drop_duration}s).")
        return True

    def execute_post_drop_maneuver(self, target_altitude=None) -> bool:
        """
        Menjalankan dorongan maneuver pasca-drop payload (misalnya 2 kali dorongan ke kiri)
        setelah servo drop selesai dan sebelum pendaratan dilakukan.
        """
        if self.post_drop_pulses <= 0:
            return True

        if target_altitude is None:
            target_altitude = get_takeoff_config().get('takeoff_altitude', 1.2)

        self.get_logger().info(
            f"[POST-DROP] Mengeksekusi {self.post_drop_pulses} kali dorongan '{self.post_drop_direction}' "
            f"(Roll PWM: {1500 + self.post_drop_roll_override}, Pitch PWM: {1500 + self.post_drop_pitch_override})..."
        )

        for step in range(1, self.post_drop_pulses + 1):
            if not rclpy.ok():
                return False

            self.get_logger().info(
                f"[POST-DROP Step {step}/{self.post_drop_pulses}] Dorong {self.post_drop_direction} selama {self.post_drop_pulse_duration}s..."
            )

            # Phase 1: Dorongan Pulsa Pasca-Drop (dengan Active Alt Hold & Servo Drop Position)
            start_pulse = time.time()
            while rclpy.ok() and (time.time() - start_pulse < self.post_drop_pulse_duration):
                self.rc_channels[0] = 1500 + self.post_drop_roll_override
                self.rc_channels[1] = 1500 + self.post_drop_pitch_override
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3
                self.rc_channels[3] = 1500
                self.set_rc_channel(self.servo_channel, self.servo_drop_pwm)

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

            # Phase 2: Pause / Delay netral antar pulsa
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[3] = 1500

            start_pause = time.time()
            while rclpy.ok() and (time.time() - start_pause < self.post_drop_pause_duration):
                rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                self.rc_channels[2] = rc3
                self.set_rc_channel(self.servo_channel, self.servo_drop_pwm)

                if not self.rc_timer:
                    self.publish_rc()

                rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("[POST-DROP] Dorongan pasca-drop selesai. Menetralkan kontrol Roll & Pitch.")
        self.rc_channels[0] = 1500
        self.rc_channels[1] = 1500
        self.rc_channels[3] = 1500
        if not self.rc_timer:
            self.publish_rc()

        return True

    def run_mission(self):
        """Menjalankan seluruh alur misi otonom."""
        # 0. Set servo ke posisi terkunci (1900 PWM) sebelum takeoff
        self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

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

        # 4. Dorongan pasca-deteksi (dorong ke kiri 2 kali sebelum drop)
        if not self.execute_post_detection_maneuver():
            self.get_logger().error("Misi dibatalkan: Maneuver pasca-deteksi terinterupsi.")
            return

        # 5. PELEPASAN PAYLOAD (SERVO DROP) - PWM 1900 -> 1100
        if not self.drop_payload():
            self.get_logger().error("Misi dibatalkan: Drop payload gagal.")
            return

        # 6. Dorongan pasca-drop (dorong ke kiri 2 kali setelah drop)
        if not self.execute_post_drop_maneuver():
            self.get_logger().error("Misi dibatalkan: Maneuver pasca-drop terinterupsi.")
            return

        # 7. Hover pasca-drop & maneuver untuk stabilisasi akhir
        if not self.hover(duration_seconds=self.hover_duration_after):
            self.get_logger().error("Misi dibatalkan: Hover akhir gagal.")
            return

        # 8. Pendaratan otonom dan cleanup
        self.get_logger().info("Misi search, overshoot, maneuver, drop servo, & post-drop maneuver selesai! Melakukan pendaratan...")
        self.land()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionSearchDropLandNode()
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
