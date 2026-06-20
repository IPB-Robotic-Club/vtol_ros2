import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

class VehicleStatusPrinter(Node):
    def __init__(self):
        super().__init__('vehicle_status_printer')

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

        # Timer to print status every 0.1 second (10 Hz) for real-time updates
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Service client for setting stream rate
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # Variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_battery = BatteryState()
        self.has_pose = False
        self.has_battery = False
        self.stream_rate_requested = False

        self.get_logger().info('Vehicle Status Printer Node Started!')

    def state_callback(self, msg):
        self.current_state = msg

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

    def timer_callback(self):
        if not self.current_state.connected:
            print("\033[H\033[2J==========================================")
            print("         VEHICLE STATUS PRINTER           ")
            print("==========================================")
            print(" Autopilot status: DISCONNECTED")
            print(" Waiting for connection...")
            print("==========================================")
            return

        # Request telemetry stream rates once connected
        if not self.stream_rate_requested:
            self.request_stream_rate()

        # Formatting values
        conn_str = "CONNECTED"
        arm_str = "ARMED" if self.current_state.armed else "DISARMED"
        mode_str = self.current_state.mode if self.current_state.mode else "UNKNOWN"
        
        # Local position
        if self.has_pose:
            p = self.current_pose.pose.position
            pos_str = f"X: {p.x:7.2f} | Y: {p.y:7.2f} | Z (Alt): {p.z:7.2f}"
        else:
            pos_str = "No local position data yet."

        # Battery
        if self.has_battery:
            volt = self.current_battery.voltage
            pct = self.current_battery.percentage * 100.0
            bat_str = f"{volt:5.2f}V ({pct:5.1f}%)"
        else:
            bat_str = "No battery data yet."

        # Print layout (clear screen first for updating display)
        print("\033[H\033[2J==========================================")
        print("         VEHICLE STATUS PRINTER           ")
        print("==========================================")
        print(f" Autopilot Conn : {conn_str}")
        print(f" Arm Status     : {arm_str}")
        print(f" Flight Mode    : {mode_str}")
        print(f" Battery        : {bat_str}")
        print("------------------------------------------")
        print(" Local Position:")
        print(f"   {pos_str}")
        print("==========================================")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusPrinter()
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
