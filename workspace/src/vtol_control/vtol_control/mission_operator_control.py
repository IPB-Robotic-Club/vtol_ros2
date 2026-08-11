import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_operator_control_config, get_takeoff_config
import time
import sys
import select

# Import termios & tty untuk non-blocking keyboard input di Linux terminal
try:
    import termios
    import tty
    TERMIOS_AVAILABLE = True
except ImportError:
    TERMIOS_AVAILABLE = False


class MissionOperatorControlNode(VtolBaseNode):
    """
    Misi 12: Mission node untuk Takeoff, Hover, dan kontrol manual RC Override via operator (keyboard).
    - WASD : Pitch Forward/Backward, Roll Left/Right
    - QE   : Yaw Left (CCW) / Yaw Right (CW)
    - F    : Drop Payload via Servo (RC CH9 PWM 1900 -> 1100)
    - L    : Pendaratan Otonom (Land)
    Durasi dan kekuatan dorongan RC diatur di vtol_config.yaml (blok operator_control).
    """
    def __init__(self):
        super().__init__('mission_operator_control_node', enable_rc_loop=True)

        # Membaca konfigurasi dari vtol_config.yaml
        op_config = get_operator_control_config()
        self.pitch_override = op_config.get('pitch_override', 60)
        self.roll_override = op_config.get('roll_override', 60)
        self.yaw_override = op_config.get('yaw_override', 50)
        self.pulse_duration = op_config.get('pulse_duration', 0.5)
        self.pause_duration = op_config.get('pause_duration', 0.5)
        self.tilt_compensation_gain = op_config.get('tilt_compensation_gain', 0.25)

        # Parameter Servo Drop
        self.servo_channel = op_config.get('servo_channel', 9)
        self.servo_initial_pwm = op_config.get('servo_initial_pwm', 1900)
        self.servo_drop_pwm = op_config.get('servo_drop_pwm', 1100)
        self.servo_drop_duration = op_config.get('servo_drop_duration', 2.0)

        # State tracking servo
        self.current_servo_pwm = self.servo_initial_pwm
        self.set_rc_channel(self.servo_channel, self.current_servo_pwm)

        self.get_logger().info(
            f"[Mission12 - OperatorControl] Node Siap! "
            f"Pitch Override=±{self.pitch_override}, Roll Override=±{self.roll_override}, "
            f"Yaw Override=±{self.yaw_override}, Pulse={self.pulse_duration}s, Pause={self.pause_duration}s, "
            f"Servo CH{self.servo_channel} Init PWM={self.servo_initial_pwm}, Drop PWM={self.servo_drop_pwm}"
        )

    def _get_key(self, settings, timeout=0.05) -> str:
        """Membaca 1 tombol keyboard dari stdin secara non-blocking."""
        if not TERMIOS_AVAILABLE or not sys.stdin.isatty():
            return ''

        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return key

    def execute_rc_pulse(self, action_name: str, roll_delta: int = 0, pitch_delta: int = 0, yaw_delta: int = 0, target_altitude: float = 1.2):
        """
        Mengirimkan dorongan pulsa RC override selama pulse_duration detik,
        diikuti oleh jeda netral selama pause_duration detik dengan active altitude hold.
        """
        self.get_logger().info(
            f"[OPERATOR ACTION] {action_name} | Roll: {1500 + roll_delta}, Pitch: {1500 + pitch_delta}, Yaw: {1500 + yaw_delta} "
            f"selama {self.pulse_duration}s..."
        )

        # ── Phase 1: Pulsa Dorongan RC Override ──
        start_pulse = time.time()
        while rclpy.ok() and (time.time() - start_pulse < self.pulse_duration):
            self.rc_channels[0] = 1500 + roll_delta
            self.rc_channels[1] = 1500 + pitch_delta
            rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
            self.rc_channels[2] = rc3
            self.rc_channels[3] = 1500 + yaw_delta
            self.set_rc_channel(self.servo_channel, self.current_servo_pwm)

            if not self.rc_timer:
                self.publish_rc()

            rclpy.spin_once(self, timeout_sec=0.05)

        # ── Phase 2: Jeda Netral Stabilisasi ──
        self.rc_channels[0] = 1500
        self.rc_channels[1] = 1500
        self.rc_channels[3] = 1500

        start_pause = time.time()
        while rclpy.ok() and (time.time() - start_pause < self.pause_duration):
            rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
            self.rc_channels[2] = rc3
            self.set_rc_channel(self.servo_channel, self.current_servo_pwm)

            if not self.rc_timer:
                self.publish_rc()

            rclpy.spin_once(self, timeout_sec=0.05)

    def drop_payload(self, target_altitude: float = 1.2):
        """Mengeksekusi pelepas beban (Servo Drop: PWM 1900 -> 1100)."""
        self.get_logger().info(
            f"\n=======================================================\n"
            f" >>> MELEPAS PAYLOAD VIA SERVO CH{self.servo_channel} (PWM: {self.current_servo_pwm} -> {self.servo_drop_pwm}) <<<\n"
            f"=======================================================\n"
        )
        self.current_servo_pwm = self.servo_drop_pwm
        self.set_rc_channel(self.servo_channel, self.current_servo_pwm)

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < self.servo_drop_duration):
            rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = rc3
            self.rc_channels[3] = 1500
            self.set_rc_channel(self.servo_channel, self.current_servo_pwm)

            if not self.rc_timer:
                self.publish_rc()

            current_alt = self.get_current_altitude()
            self.get_logger().info(
                f"[SERVO DROP] Holding altitude: {current_alt:.2f}m/{target_altitude:.2f}m | "
                f"CH{self.servo_channel} PWM: {self.current_servo_pwm} | elapsed: {time.time() - start_time:.1f}s",
                throttle_duration_sec=0.5
            )
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"[SERVO DROP] Payload berhasil dilepas! (PWM CH{self.servo_channel}: {self.servo_drop_pwm}).")

    def run_operator_loop(self, target_altitude: float = 1.2):
        """Loop interaktif untuk membaca input tombol dari operator."""
        old_settings = None
        if TERMIOS_AVAILABLE and sys.stdin.isatty():
            old_settings = termios.tcgetattr(sys.stdin)

        print("\n" + "=" * 60, flush=True)
        print("    OPERATOR MANUAL RC OVERRIDE CONTROL TERHUBUNG", flush=True)
        print("=" * 60, flush=True)
        print("  W / w : Pitch Forward  (Maju)", flush=True)
        print("  S / s : Pitch Backward (Mundur)", flush=True)
        print("  A / a : Roll Left      (Kiri)", flush=True)
        print("  D / d : Roll Right     (Kanan)", flush=True)
        print("  Q / q : Yaw Left       (CCW / Putar Kiri)", flush=True)
        print("  E / e : Yaw Right      (CW / Putar Kanan)", flush=True)
        print("  F / f : Drop Payload   (Servo CH9 1900 -> 1100 PWM)", flush=True)
        print("  L / l : Land           (Pendaratan Otonom)", flush=True)
        print("  Ctrl+C: Abort Flight & Emergency Landing", flush=True)
        print("=" * 60 + "\n", flush=True)

        try:
            while rclpy.ok():
                # Spin ROS2 node untuk memproses telemetry & RC loop
                rclpy.spin_once(self, timeout_sec=0.05)

                # Watchdog check: koneksi MAVROS
                if self.state_received and not self.current_state.connected:
                    self.get_logger().error("Autopilot terputus saat kontrol operator! Aborting mission.")
                    self.abort_flight()
                    return

                # Watchdog check: mode penerbangan
                if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                    self.get_logger().warn(
                        f"Manual override terdeteksi! Mode penerbangan berubah ke {self.current_state.mode}. Aborting mission."
                    )
                    self.abort_flight()
                    return

                # Membaca input tombol dari operator
                key = self._get_key(old_settings, timeout=0.05)

                if not key:
                    # Pertahankan altitude hold & servo PWM saat tidak ada input
                    rc3 = self.compute_altitude_hold_rc3(target_altitude, self.tilt_compensation_gain)
                    self.rc_channels[0] = 1500
                    self.rc_channels[1] = 1500
                    self.rc_channels[2] = rc3
                    self.rc_channels[3] = 1500
                    self.set_rc_channel(self.servo_channel, self.current_servo_pwm)
                    if not self.rc_timer:
                        self.publish_rc()
                    continue

                key_lower = key.lower()

                if key_lower == 'w':
                    # Pitch Forward (Pitch negative = Forward di ArduPilot)
                    self.execute_rc_pulse("PITCH FORWARD (W)", pitch_delta=-self.pitch_override, target_altitude=target_altitude)
                elif key_lower == 's':
                    # Pitch Backward (Pitch positive = Backward)
                    self.execute_rc_pulse("PITCH BACKWARD (S)", pitch_delta=self.pitch_override, target_altitude=target_altitude)
                elif key_lower == 'a':
                    # Roll Left (Roll negative = Left)
                    self.execute_rc_pulse("ROLL LEFT (A)", roll_delta=-self.roll_override, target_altitude=target_altitude)
                elif key_lower == 'd':
                    # Roll Right (Roll positive = Right)
                    self.execute_rc_pulse("ROLL RIGHT (D)", roll_delta=self.roll_override, target_altitude=target_altitude)
                elif key_lower == 'q':
                    # Yaw Left / CCW (Yaw negative = CCW)
                    self.execute_rc_pulse("YAW LEFT CCW (Q)", yaw_delta=-self.yaw_override, target_altitude=target_altitude)
                elif key_lower == 'e':
                    # Yaw Right / CW (Yaw positive = CW)
                    self.execute_rc_pulse("YAW RIGHT CW (E)", yaw_delta=self.yaw_override, target_altitude=target_altitude)
                elif key_lower == 'f':
                    # Drop Servo Payload
                    self.drop_payload(target_altitude=target_altitude)
                elif key_lower == 'l':
                    # Land otonom
                    self.get_logger().info("[OPERATOR ACTION] Perintah LAND (L) diterima dari operator! Memulai pendaratan...")
                    break
                else:
                    self.get_logger().info(f"Tombol '{key}' tidak dikenal. Gunakan WASD, QE, F (drop), atau L (land).")

        finally:
            if old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run_mission(self):
        """Menjalankan alur Misi 12."""
        target_alt = get_takeoff_config().get('takeoff_altitude', 1.2)

        # 0. Lock Servo ke posisi awal (1900 PWM)
        self.set_rc_channel(self.servo_channel, self.servo_initial_pwm)

        # 1. Takeoff otonom ke target altitude
        if not self.takeoff(target_altitude=target_alt):
            self.get_logger().error("Misi dibatalkan: Takeoff gagal.")
            return

        # 2. Hover stabilisasi awal
        if not self.hover(duration_seconds=2.0):
            self.get_logger().error("Misi dibatalkan: Hover awal gagal.")
            return

        # 3. Serahkan kontrol ke Operator via keyboard interaktif
        self.run_operator_loop(target_altitude=target_alt)

        # 4. Pendaratan otonom
        self.get_logger().info("Kontrol operator selesai. Melakukan pendaratan otonom...")
        self.land()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionOperatorControlNode()
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
