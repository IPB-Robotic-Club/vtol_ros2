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
        from vtol_control.config_reader import get_active_profile
        self._active_profile = get_active_profile()
        # Sumber altitude: False = local_position/pose.z (SITL), True = rangefinder_1 (real)
        self.use_rangefinder = (self._active_profile == 'serial')

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

        # Subscription ke rangefinder_1 hanya aktif untuk drone real (profil 'serial')
        if self.use_rangefinder:
            self.get_logger().info("[AltSource] Profil 'serial' terdeteksi — menggunakan RANGEFINDER_1 sebagai sumber altitude.")
            self.rangefinder_sub = self.create_subscription(
                Range,
                '/mavros/rangefinder_1/range',
                self._rangefinder_callback,
                self.qos_telemetry
            )
        else:
            self.get_logger().info("[AltSource] Profil 'tcp' terdeteksi — menggunakan LOCAL_POSITION/POSE.Z sebagai sumber altitude.")
            self.rangefinder_sub = None

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
            return self.rangefinder_range if self.has_rangefinder else 0.0
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
            return True
        else:
            self.get_logger().error(f"Arming service not available for value: {arm_value}")
            return False

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

    def takeoff(self, target_altitude=None, throttle=None, timeout=20.0):
        """Synchronously commands takeoff to target_altitude using RC overrides."""
        from vtol_control.config_reader import get_takeoff_config
        config = get_takeoff_config()
        if target_altitude is None:
            target_altitude = config.get('takeoff_altitude', 1.5)
        if throttle is None:
            throttle = config.get('takeoff_throttle', 1700)

        self.get_logger().info(f"Starting Takeoff to {target_altitude}m with throttle {throttle}...")
        
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
            
        # 2. Arm the drone
        self.get_logger().info("Arming drone...")
        self.set_arm(True)
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < 5.0:
            if self.current_state.armed:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error("Takeoff aborted: Failed to ARM drone.")
            return False

        # 3. Climb
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
                self.get_logger().info(f"Target altitude reached ({current_alt:.2f}m >= {target_altitude}m).")
                return True
                
            if elapsed > timeout:
                self.get_logger().error(f"Takeoff timeout! Failed to reach {target_altitude}m in {timeout}s. Current alt: {current_alt:.2f}m. Aborting.")
                self.abort_flight()
                return False
                
            self.get_logger().info(f"Climbing... alt: {current_alt:.2f}m/{target_altitude}m, throttle: {throttle}, elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return False

    def hover(self, duration_seconds):
        """Synchronously hovers for duration_seconds by holding neutral throttle."""
        self.get_logger().info(f"Entering Hover phase for {duration_seconds} seconds...")
        self.rc_channels[0] = 1500
        self.rc_channels[1] = 1500
        self.rc_channels[2] = 1500 # Neutral throttle
        self.rc_channels[3] = 1500
        if not self.rc_timer:
            self.publish_rc()

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
                self.get_logger().info(f"Hover complete ({duration_seconds}s).")
                return True
                
            self.get_logger().info(f"Hovering... throttle: 1500, elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)
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
