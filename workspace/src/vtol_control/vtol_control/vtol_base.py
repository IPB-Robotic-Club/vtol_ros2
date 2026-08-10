import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import BatteryState, Range
from geometry_msgs.msg import PoseStamped
import time

class VtolBaseNode(Node):
    def __init__(self, node_name, enable_rc_loop=False):
        super().__init__(node_name)

        # QoS Profiles
        self.qos_telemetry = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Deteksi profil aktif: 'tcp' = SITL, 'serial' = drone real
        from vtol_control.config_reader import get_active_profile, get_takeoff_config, get_pid_config
        self._active_profile = get_active_profile()
        # Sumber altitude: False = local_position/pose.z (SITL), True = rangefinder_1 (real)
        self.use_rangefinder = (self._active_profile == 'serial')

        takeoff_config = get_takeoff_config()
        self.max_throttle_override = takeoff_config.get('max_throttle_override', 30)

        # Active altitude hold parameters (terpusat di VtolBaseNode)
        pid_config = get_pid_config()
        self.alt_hold_enabled = pid_config.get('hold_altitude', True)
        self.kp_altitude = pid_config.get('kp_altitude', 80.0)
        self.hover_base = pid_config.get('hover_base', 1576)
        self.max_alt_correction = pid_config.get('max_throttle_correction', 150)

        # Telemetry State Variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_battery = BatteryState()
        self.has_pose = False
        self.has_battery = False
        self.state_received = False
        self.last_state_time = 0.0

        # Rangefinder (hanya aktif saat profil 'serial' / drone real)
        self.rangefinder_range = 0.0
        self.has_rangefinder = False

        # Buffer for RC channels: 18 channels, default 0 (no override)
        self.rc_channels = [0] * 18

        # Subscriptions
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            self.qos_state
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            self.qos_telemetry
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            self.qos_telemetry
        )

        # Subscription ke rangefinder_1 (selalu diawasi agar telemetry rangefinder selalu tersedia)
        self.rangefinder_sub = self.create_subscription(
            Range,
            '/mavros/rangefinder/rangefinder',
            self._rangefinder_callback,
            self.qos_telemetry
        )
        if self.use_rangefinder:
            self.get_logger().info("[AltSource] Profil 'serial' terdeteksi — menggunakan RANGEFINDER (/mavros/rangefinder/rangefinder) sebagai sumber altitude utama.")
        else:
            self.get_logger().info("[AltSource] Profil 'tcp' terdeteksi — menggunakan LOCAL_POSITION/POSE.Z sebagai sumber altitude utama.")

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Publisher for MAVROS RC overrides
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Background RC publisher timer (10 Hz)
        if enable_rc_loop:
            self.rc_timer = self.create_timer(0.1, self.publish_rc)
        else:
            self.rc_timer = None

    def _state_callback(self, msg):
        self.current_state = msg
        self.last_state_time = self.get_clock().now().nanoseconds / 1e9
        self.state_received = True
        self.on_state(msg)

    def _pose_callback(self, msg):
        self.current_pose = msg
        self.has_pose = True
        self.on_pose(msg)

    def _battery_callback(self, msg):
        self.current_battery = msg
        self.has_battery = True
        self.on_battery(msg)

    def _rangefinder_callback(self, msg):
        # Abaikan pembacaan invalid (0.0 atau di luar range sensor)
        if msg.range > msg.min_range and msg.range < msg.max_range:
            self.rangefinder_range = msg.range
            self.has_rangefinder = True
            self.on_rangefinder(msg)

    # Virtual callbacks to override
    def on_state(self, msg):
        pass

    def on_pose(self, msg):
        pass

    def on_battery(self, msg):
        pass

    def on_rangefinder(self, msg):
        pass

    def get_current_altitude(self) -> float:
        """
        Mengembalikan ketinggian saat ini berdasarkan profil aktif:
        - 'tcp'    (SITL)       : local_position/pose.z (EKF)
        - 'serial' (real drone) : rangefinder_1 range (AGL, lebih akurat untuk landing)
        """
        if self.use_rangefinder:
            if self.has_rangefinder:
                return self.rangefinder_range
            return self.current_pose.pose.position.z if self.has_pose else 0.0
        else:
            return self.current_pose.pose.position.z if self.has_pose else 0.0

    # Reusable control functions
    def change_mode(self, mode_name):
        if self.set_mode_client.wait_for_service(timeout_sec=0.5):
            req = SetMode.Request()
            req.custom_mode = mode_name
            self.set_mode_client.call_async(req)
            return True
        else:
            self.get_logger().error(f"SetMode service not available for mode: {mode_name}")
            return False

    def set_arm(self, arm_value):
        if self.arming_client.wait_for_service(timeout_sec=0.5):
            req = CommandBool.Request()
            req.value = arm_value
            self.arming_client.call_async(req)
        else:
            self.get_logger().error(f"Arming service not available for value: {arm_value}")
            return False

    def wait_for_operator_confirmation(self, prompt="Tekan [ENTER] untuk melanjutkan ke instruksi selanjutnya...") -> bool:
        """
        Meminta konfirmasi dari operator via tombol Enter di terminal (stdin).
        Tetap menjalankan rclpy.spin_once() agar callback MAVROS, telemetry, dan watchdog tetap aktif.
        """
        import select
        import sys

        self.get_logger().info(f"[CONFIRMATION REQUIRED] {prompt}")
        print(f"\n=======================================================", flush=True)
        print(f" >>> KONFIRMASI OPERATOR: {prompt} <<<", flush=True)
        print(f"=======================================================\n", flush=True)

        while rclpy.ok():
            # Spin ROS2 node agar telemetry dan RC timer tetap aktif
            rclpy.spin_once(self, timeout_sec=0.05)

            # Watchdog check: koneksi MAVROS
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot terputus saat menunggu konfirmasi operator! Aborting.")
                return False

            # Watchdog check: jika ter-disarm secara tidak terduga saat menunggu
            if not self.current_state.armed:
                self.get_logger().error("Drone ter-DISARM saat menunggu konfirmasi operator! Aborting.")
                return False

            # Cek ketersediaan stdin tanpa blocking
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                if rlist:
                    line = sys.stdin.readline()
                    if line == "":  # Non-interactive stdin (EOF)
                        self.get_logger().warn("Lingkungan non-interaktif terdeteksi (stdin EOF). Otomatis melanjutkan...")
                        return True
                    self.get_logger().info("Konfirmasi operator diterima! Melanjutkan ke instruksi selanjutnya.")
                    return True
            except Exception as e:
                self.get_logger().warn(f"Gagal membaca stdin ({e}). Otomatis melanjutkan...")
                return True

        return False

    def arm(self, timeout=5.0, confirm=True) -> bool:
        """
        Mengirim perintah ARM dan menunggu konfirmasi status armed.
        Jika confirm=True, meminta konfirmasi ulang operator (tombol Enter) sebelum melanjutkan.
        """
        self.get_logger().info("Arming drone...")
        self.set_arm(True)
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout:
            if self.current_state.armed:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error("Failed to ARM drone.")
            return False

        self.get_logger().info("Drone successfully ARMED!")

        if confirm:
            return self.wait_for_operator_confirmation("Drone ARMED! Tekan [ENTER] untuk melanjutkan ke instruksi selanjutnya...")
        return True

    def set_rc_channel(self, channel: int, pwm_value: int):
        """
        Set override value for a specific RC channel (1-18).
        channel: 1-indexed (e.g. 9 for CH9)
        pwm_value: PWM value in microseconds (e.g. 1100, 1500, 1900) or 0 to release
        """
        if 1 <= channel <= 18:
            self.rc_channels[channel - 1] = int(pwm_value)
        else:
            self.get_logger().error(f"Invalid RC channel: {channel}. Must be between 1 and 18.")

    def publish_rc(self):
        # MAVROS Connection watchdog
        if self.state_received and ((self.get_clock().now().nanoseconds / 1e9) - self.last_state_time > 3.0):
            if self.current_state.connected:
                self.get_logger().error("MAVROS connection lost! Zeroing RC overrides and marking as disconnected.")
                self.current_state.connected = False
            self.rc_channels = [0] * 18
            return

        msg = OverrideRCIn()
        msg.channels = self.rc_channels
        self.rc_pub.publish(msg)

    def takeoff(self, target_altitude=None, throttle=None, timeout=20.0, confirm=True):
        """Synchronously commands takeoff to target_altitude using RC overrides."""
        from vtol_control.config_reader import get_takeoff_config
        config = get_takeoff_config()
        if target_altitude is None:
            target_altitude = config.get('takeoff_altitude', 1.5)
        if throttle is None:
            throttle = config.get('takeoff_throttle', 1500 + self.max_throttle_override)

        self.get_logger().info(f"Starting Takeoff to {target_altitude}m with throttle {throttle} (1500 + {throttle - 1500})...")
        
        # Verify connection first
        while rclpy.ok() and not self.state_received:
            self.get_logger().info("Waiting for MAVROS state messages...", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok() and not self.current_state.connected:
            self.get_logger().info("Waiting for autopilot connection...", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Change mode to LOITER
        self.get_logger().info("Changing mode to LOITER...")
        self.change_mode("LOITER")
        
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < 5.0:
            if self.current_state.mode in ["LOITER", "CMODE(5)"]:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error("Takeoff aborted: Failed to change mode to LOITER.")
            return False
            
        # 2. Arm the drone & wait for operator confirmation
        if not self.arm(timeout=5.0, confirm=confirm):
            self.get_logger().error("Takeoff aborted: Arming or operator confirmation failed.")
            return False

        # 3. Climb
        # Slow-down zone: throttle dikurangi proporsional saat mendekati target altitude
        # untuk mencegah LOITER altitude controller overshoot/osilasi fighting dengan RC override.
        SLOWDOWN_ZONE = 0.1   # meter sebelum target mulai kurangi throttle
        THROTTLE_MIN  = 1515  # throttle minimal saat di dalam slow-down zone (cukup untuk loft halus)
        throttle_range = throttle - THROTTLE_MIN  # rentang throttle dari min ke full climb

        self.get_logger().info("Drone successfully ARMED! Climbing...")
        self.rc_channels[0] = 1500 # Roll
        self.rc_channels[1] = 1500 # Pitch
        self.rc_channels[2] = throttle # Throttle (Climb)
        self.rc_channels[3] = 1500 # Yaw
        if not self.rc_timer:
            self.publish_rc()

        start_time = time.time()
        while rclpy.ok():
            current_time = time.time()
            elapsed = current_time - start_time

            # Watchdog check: connection
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected during takeoff! Aborting.")
                self.abort_flight()
                return False

            # Watchdog check: manual override
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(f"Manual override detected during takeoff! Flight mode changed to {self.current_state.mode}. Aborting flight.")
                self.abort_flight()
                return False

            current_alt = self.get_current_altitude()

            if current_alt >= target_altitude:
                self.get_logger().info(f"Target altitude reached ({current_alt:.2f}m >= {target_altitude}m). Neutralizing throttle.")
                # Netralkan throttle segera agar tidak ada jeda climb sebelum hover/PID mengambil alih
                self.rc_channels[2] = 1500
                return True

            if elapsed > timeout:
                self.get_logger().error(f"Takeoff timeout! Failed to reach {target_altitude}m in {timeout}s. Current alt: {current_alt:.2f}m. Aborting.")
                self.abort_flight()
                return False

            # Slow-down zone: kurangi throttle proporsional saat mendekati target
            altitude_remaining = target_altitude - current_alt
            if altitude_remaining <= SLOWDOWN_ZONE and throttle_range > 0:
                # Linear interpolasi: 0m sisa → THROTTLE_MIN, SLOWDOWN_ZONE sisa → throttle penuh
                ratio = altitude_remaining / SLOWDOWN_ZONE
                effective_throttle = int(THROTTLE_MIN + ratio * throttle_range)
            else:
                effective_throttle = throttle

            self.rc_channels[2] = effective_throttle
            self.get_logger().info(
                f"Climbing... alt: {current_alt:.2f}m/{target_altitude}m, "
                f"throttle: {effective_throttle}, elapsed: {elapsed:.1f}s",
                throttle_duration_sec=1.0
            )
            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def compute_altitude_hold_rc3(self, target_altitude=None) -> int:
        """
        Hitung nilai RC3 (throttle) untuk active altitude hold menggunakan P-controller.
        Tujuan: mempertahankan target_altitude selama fase hover/centering.

        Formula: RC3 = hover_base + kp_altitude × (target_alt − current_alt)
        Clamped by max_alt_correction.
        """
        if not self.alt_hold_enabled:
            return 1500

        if target_altitude is None:
            from vtol_control.config_reader import get_takeoff_config
            target_altitude = get_takeoff_config().get('altitude', 1.5)

        current_alt = self.get_current_altitude()
        alt_error = target_altitude - current_alt
        correction = self.kp_altitude * alt_error
        correction = max(min(correction, self.max_alt_correction), -self.max_alt_correction)
        return int(self.hover_base + correction)

    def hover(self, duration_seconds, target_altitude=None):
        """Synchronously hovers for duration_seconds with active altitude hold."""
        if target_altitude is None:
            from vtol_control.config_reader import get_takeoff_config
            target_altitude = get_takeoff_config().get('altitude', 1.5)

        self.get_logger().info(f"Entering Hover phase for {duration_seconds} seconds (target altitude: {target_altitude:.2f}m)...")

        start_time = time.time()
        while rclpy.ok():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Watchdog check: connection
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected during hover! Aborting.")
                self.abort_flight()
                return False

            # Watchdog check: manual override
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(f"Manual override detected during hover! Flight mode changed to {self.current_state.mode}. Aborting flight.")
                self.abort_flight()
                return False
                
            if elapsed >= duration_seconds:
                self.get_logger().info(f"Hover complete ({duration_seconds}s). Alt akhir: {self.get_current_altitude():.3f}m")
                return True
                
            rc3 = self.compute_altitude_hold_rc3(target_altitude)
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = rc3
            self.rc_channels[3] = 1500
            if not self.rc_timer:
                self.publish_rc()

            current_alt = self.get_current_altitude()
            self.get_logger().info(
                f"[HOVER] alt: {current_alt:.2f}m/{target_altitude:.2f}m | "
                f"err: {target_altitude - current_alt:+.3f}m | RC3: {rc3} | elapsed: {elapsed:.1f}s",
                throttle_duration_sec=1.0
            )
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return False

    def land(self, timeout=30.0):
        """Synchronously commands LAND mode and releases all overrides."""
        self.get_logger().info("Switching mode to LAND for graceful landing...")
        self.change_mode("LAND")
        self.rc_channels = [0] * 18
        self.publish_rc()

        start_time = time.time()
        while rclpy.ok():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Watchdog check: connection
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected during landing!")
                return False

            if not self.current_state.armed:
                self.get_logger().info("Drone successfully DISARMED on ground. Landing complete!")
                self._post_land_cleanup()
                return True
                
            if elapsed > timeout:
                self.get_logger().error(f"Landing timeout! Drone failed to disarm within {timeout}s.")
                return False
                
            # If mode changed from LAND, retry landing command
            if self.current_state.mode not in ["LAND", "CMODE(9)"] and elapsed > 5.0:
                self.get_logger().warn("LAND mode interrupted, retrying LAND command...")
                self.change_mode("LAND")
                
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return False

    def _post_land_cleanup(self):
        """
        Reset FCU ke state netral setelah landing selesai.
        Mencegah 'Mission is stale' pada sesi arming berikutnya:
        ArduPilot menandai mission sebagai stale jika FCU tidak di-reset
        ke mode netral setelah LAND mode selesai.
        """
        self.get_logger().info("[PostLand] Mereset FCU ke mode STABILIZE...")
        # STABILIZE tidak membutuhkan mission — menghilangkan 'mission stale' check
        self.change_mode("STABILIZE")
        # Pastikan semua RC override dilepas
        self.rc_channels = [0] * 18
        # Tunggu sebentar agar perintah mode change terkirim ke FCU
        start = time.time()
        while rclpy.ok() and time.time() - start < 1.5:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("[PostLand] Cleanup selesai. FCU siap untuk sesi berikutnya.")

    def abort_flight(self):
        """Immediately aborts flight, releases control, and commands LAND."""
        self.get_logger().error("FLIGHT ABORTED: Releasing overrides and commanding immediate LAND!")
        self.rc_channels = [0] * 18
        try:
            self.publish_rc()
        except Exception as e:
            self.get_logger().warn(f"Failed to publish safety RC release: {e}")
            
        self.change_mode("LAND")
        
        # Spin briefly to allow the async mode change service call to be sent
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < 1.5:
            rclpy.spin_once(self, timeout_sec=0.1)

    def safe_exit(self):
        """Executes flight abort sequence if node is interrupted while armed."""
        if self.state_received and self.current_state.armed:
            self.get_logger().warn("KeyboardInterrupt detected! Performing safety landing sequence...")
            self.abort_flight()
