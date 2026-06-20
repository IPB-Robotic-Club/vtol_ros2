import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate

class VtolCore(Node):
    def __init__(self):
        super().__init__('vtol_core')

        self.autopilot_connected = False
        self.stream_rate_requested = False
        self.armed_state = False
        self.flight_mode = None

        # Define QoS Profile for MAVROS State
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        self.qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # State subscription
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            self.qos_state
        )

        # Service client for stream rate request
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        self.get_logger().info('VTOL Core Node Started (Basic Mode)!')

    def state_callback(self, msg):
        # 1. Log connection status changes
        if msg.connected != self.autopilot_connected:
            self.autopilot_connected = msg.connected
            if self.autopilot_connected:
                self.get_logger().info("Autopilot Link Connected successfully!")
            else:
                self.get_logger().warn("Autopilot Link Connection lost!")

        # 2. Log arm status changes
        if msg.armed != self.armed_state:
            self.armed_state = msg.armed
            status = "ARMED" if self.armed_state else "DISARMED"
            self.get_logger().info(f"Drone status changed to: {status}")

        # 3. Log flight mode changes
        if msg.mode != self.flight_mode:
            self.flight_mode = msg.mode
            self.get_logger().info(f"Flight Mode changed to: {self.flight_mode}")

        # 4. Request stream rate once connected
        if self.autopilot_connected and not self.stream_rate_requested:
            self.request_stream_rate()

    def request_stream_rate(self):
        if self.stream_rate_client.wait_for_service(timeout_sec=1.0):
            req = StreamRate.Request()
            req.stream_id = 0  # STREAM_ALL
            req.message_rate = 10
            req.on_off = True
            self.stream_rate_client.call_async(req)
            self.stream_rate_requested = True
            self.get_logger().info("Requested autopilot telemetry stream rate at 10Hz.")
        else:
            self.get_logger().warn("SetMode / StreamRate service not available yet, will retry.")

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
