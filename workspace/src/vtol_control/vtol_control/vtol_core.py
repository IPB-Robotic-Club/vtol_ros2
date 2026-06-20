import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, SetMode
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
import time

class VtolCore(Node):
    def __init__(self):
        super().__init__('vtol_core')

        # Define QoS Profiles
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

        # State subscriptions
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            self.qos_state
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            self.qos_telemetry
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            self.qos_telemetry
        )

        # Timer for HUD and Failsafe Watchdog (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Service clients
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Telemetry State Variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_battery = BatteryState()
        self.has_pose = False
        self.has_battery = False
        
        # Failsafe and Watchdog Variables
        self.last_state_time = 0.0  # Unix timestamp of last received MAVROS state message
        self.stream_rate_requested = False
        self.mavros_heartbeat_ok = False
        self.autopilot_connected = False
        
        # Safety overrides
        self.mission_completed = False
        self.pilot_override_active = False

        self.get_logger().info('VTOL Core Node Started!')

    def state_callback(self, msg):
        self.current_state = msg
        self.last_state_time = time.time()
        self.mavros_heartbeat_ok = True

        # Check connection status
        self.autopilot_connected = msg.connected

        # Detect GCS or Pilot manual override to LAND mode
        if msg.mode == "LAND" and not self.mission_completed:
            self.mission_completed = True
            self.pilot_override_active = True
            self.get_logger().info("GCS/Pilot manual override to LAND mode detected. Mission completed.")

    def pose_callback(self, msg):
        self.current_pose = msg
        self.has_pose = True

    def battery_callback(self, msg):
        self.current_battery = msg
        self.has_battery = True

    def request_stream_rate(self):
        if self.stream_rate_client.wait_for_service(timeout_sec=0.5):
            req = StreamRate.Request()
            req.stream_id = 0  # STREAM_ALL
            req.message_rate = 10
            req.on_off = True
            self.stream_rate_client.call_async(req)
            self.stream_rate_requested = True
            self.get_logger().info("Requested autopilot telemetry stream rate at 10Hz.")

    def trigger_failsafe_land(self):
        """Attempts to command LAND mode as a critical safety failsafe."""
        self.get_logger().error("FAILSAFE TRIGGERED: Ordering immediate LAND!")
        if self.set_mode_client.wait_for_service(timeout_sec=0.5):
            req = SetMode.Request()
            req.custom_mode = "LAND"
            self.set_mode_client.call_async(req)
        else:
            self.get_logger().error("Failsafe LAND service call failed: SetMode service not available.")

    def timer_callback(self):
        current_time = time.time()

        # 1. Watchdog: Check MAVROS Node Health (Heartbeat)
        # Only check and trigger failsafe if the drone is armed (flying)
        if self.current_state.armed:
            if self.last_state_time > 0.0:
                time_since_last_state = current_time - self.last_state_time
                if time_since_last_state > 3.0:
                    if self.mavros_heartbeat_ok:
                        self.get_logger().error("Watchdog: MAVROS Heartbeat Lost during flight! (No state messages)")
                    self.mavros_heartbeat_ok = False
                    self.trigger_failsafe_land()
        else:
            # If not armed, keep resetting the watchdog health state silently
            if self.last_state_time > 0.0:
                time_since_last_state = current_time - self.last_state_time
                self.mavros_heartbeat_ok = (time_since_last_state <= 3.0)
            else:
                self.mavros_heartbeat_ok = False

        # 2. Watchdog: Check Autopilot Connection Health
        if self.mavros_heartbeat_ok and not self.autopilot_connected:
            # MAVROS is running, but autopilot connection lost during armed state
            if self.current_state.armed:
                self.get_logger().error("Watchdog: Autopilot Link Connection Lost during flight!")
                self.trigger_failsafe_land()

        # 3. Request telemetry stream rates once connected
        if self.autopilot_connected and not self.stream_rate_requested:
            self.request_stream_rate()

def main(args=None):
    rclpy.init(args=args)
    node = VtolCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
