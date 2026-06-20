import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
import time

class VtolBaseNode(Node):
    def __init__(self, node_name):
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

        # Telemetry State Variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_battery = BatteryState()
        self.has_pose = False
        self.has_battery = False
        self.state_received = False
        self.last_state_time = 0.0

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

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

    def _state_callback(self, msg):
        self.current_state = msg
        self.last_state_time = time.time()
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

    # Virtual callbacks to override
    def on_state(self, msg):
        pass

    def on_pose(self, msg):
        pass

    def on_battery(self, msg):
        pass

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
