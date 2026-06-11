import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong, StreamRate
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

import time
import math
from threading import Thread

class DroneController(Node):
    """
    DroneController abstracts MAVROS topics and services into a clean,
    easy-to-use ROS 2 API for controlling an autonomous drone (ArduPilot/PX4).
    """
    def __init__(self, node_name='drone_controller'):
        super().__init__(node_name)
        
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

        # Telemetry State Variables
        self.current_state = State()
        self.local_pose = PoseStamped()
        self.local_velocity = TwistStamped()
        self.global_position = NavSatFix()
        
        # Control Targets
        self.target_pose = None
        self.target_velocity = None
        self.control_mode = 'POSITION'  # 'POSITION' or 'VELOCITY'
        
        # Subscriptions
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            self.qos_state
        )
        
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._local_pose_callback,
            self.qos_telemetry
        )
        
        self.local_velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._local_velocity_callback,
            self.qos_telemetry
        )
        
        self.global_position_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._global_position_callback,
            self.qos_telemetry
        )

        # Publishers
        self.local_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.local_velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel_local',
            10
        )

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # Wait for MAVROS Services to be available
        self.get_logger().info("Waiting for MAVROS services...")
        self.wait_for_services()
        self.get_logger().info("MAVROS services are available!")

        # Timer for setpoint streaming (20 Hz)
        self.stream_timer = self.create_timer(0.05, self._stream_setpoints_callback)
        self.get_logger().info("Setpoint streaming started at 20Hz.")

    def wait_for_services(self):
        """Wait until all critical MAVROS service servers are up."""
        while rclpy.ok():
            if (self.arming_client.wait_for_service(timeout_sec=1.0) and
                self.set_mode_client.wait_for_service(timeout_sec=1.0) and
                self.takeoff_client.wait_for_service(timeout_sec=1.0) and
                self.land_client.wait_for_service(timeout_sec=1.0) and
                self.command_client.wait_for_service(timeout_sec=1.0) and
                self.stream_rate_client.wait_for_service(timeout_sec=1.0)):
                break
            self.get_logger().info("Still waiting for MAVROS service servers...")

    # Callbacks
    def _state_callback(self, msg):
        self.current_state = msg

    def _local_pose_callback(self, msg):
        self.local_pose = msg

    def _local_velocity_callback(self, msg):
        self.local_velocity = msg

    def _global_position_callback(self, msg):
        self.global_position = msg

    def _stream_setpoints_callback(self):
        """Streams targets to the autopilot. Failsafe protection."""
        if not self.current_state.connected:
            return

        if self.control_mode == 'POSITION':
            # ONLY publish if we have an active target position.
            # Otherwise, do not publish to avoid fighting autopilot takeoff/landing commands.
            if self.target_pose is not None:
                pose_msg = PoseStamped()
                pose_msg.header = Header()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose = self.target_pose
                self.local_pose_pub.publish(pose_msg)

        elif self.control_mode == 'VELOCITY':
            # ONLY publish if we have an active target velocity.
            if self.target_velocity is not None:
                vel_msg = TwistStamped()
                vel_msg.header = Header()
                vel_msg.header.stamp = self.get_clock().now().to_msg()
                vel_msg.header.frame_id = "base_link"
                vel_msg.twist = self.target_velocity
                self.local_velocity_pub.publish(vel_msg)

    # Control Actions (Async wrappers)
    def arm(self, arm_state: bool):
        """Sends Arm/Disarm request to the autopilot."""
        self.get_logger().info(f"Requesting arm: {arm_state}")
        request = CommandBool.Request()
        request.value = arm_state
        
        future = self.arming_client.call_async(request)
        return future

    def set_mode(self, custom_mode: str):
        """Changes the autopilot flight mode (e.g. 'GUIDED', 'QLOITER', 'RTL')."""
        self.get_logger().info(f"Requesting flight mode: {custom_mode}")
        request = SetMode.Request()
        request.custom_mode = custom_mode
        
        future = self.set_mode_client.call_async(request)
        return future

    def takeoff(self, altitude: float, latitude: float = 0.0, longitude: float = 0.0):
        """Trigger takeoff service."""
        self.get_logger().info(f"Requesting takeoff to {altitude}m...")
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = latitude
        request.longitude = longitude
        
        future = self.takeoff_client.call_async(request)
        return future

    def vtol_takeoff(self, altitude: float, latitude: float = 0.0, longitude: float = 0.0):
        """Sends MAV_CMD_NAV_VTOL_TAKEOFF (221) command to the autopilot."""
        self.get_logger().info(f"Requesting VTOL takeoff to {altitude}m...")
        request = CommandLong.Request()
        request.command = 221  # MAV_CMD_NAV_VTOL_TAKEOFF
        request.param1 = 0.0  # Pitch
        request.param2 = 0.0
        request.param3 = 0.0
        request.param4 = 0.0  # Yaw
        request.param5 = latitude
        request.param6 = longitude
        request.param7 = altitude
        
        future = self.command_client.call_async(request)
        return future

    def land(self, latitude: float = 0.0, longitude: float = 0.0):
        """Trigger land service."""
        self.get_logger().info("Requesting landing...")
        request = CommandTOL.Request()
        request.latitude = latitude
        request.longitude = longitude
        
        future = self.land_client.call_async(request)
        return future

    def set_stream_rate(self, stream_id: int, rate: int):
        """Sets telemetry stream rate using /mavros/set_stream_rate."""
        self.get_logger().info(f"Setting stream rate for ID {stream_id} to {rate}Hz...")
        request = StreamRate.Request()
        request.stream_id = stream_id
        request.message_rate = rate
        request.on_off = True
        
        future = self.stream_rate_client.call_async(request)
        return future

    # High-level Setpoint setters
    def set_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Sets target local position in meters."""
        self.control_mode = 'POSITION'
        
        # Convert Euler yaw to Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        q = Quaternion()
        q.w = cy
        q.x = 0.0
        q.y = 0.0
        q.z = sy

        p = PoseStamped().pose
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation = q
        
        self.target_pose = p

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Sets target local velocity in m/s and yaw rate in rad/s."""
        self.control_mode = 'VELOCITY'
        
        t = TwistStamped().twist
        t.linear.x = vx
        t.linear.y = vy
        t.linear.z = vz
        t.angular.z = yaw_rate
        
        self.target_velocity = t

    def get_distance_to_target(self) -> float:
        """Returns Euclidean distance to the current position target."""
        if self.target_pose is None:
            return float('inf')
        
        dx = self.target_pose.position.x - self.local_pose.pose.position.x
        dy = self.target_pose.position.y - self.local_pose.pose.position.y
        dz = self.target_pose.position.z - self.local_pose.pose.position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
