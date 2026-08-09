import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode

class MissionServoNode(VtolBaseNode):
    def __init__(self):
        # Enable background 10Hz RC publishing timer for overrides
        super().__init__('mission_servo_node', enable_rc_loop=True)
        self.get_logger().info("Mission Servo Node Started! (Target: RC Channel 9, PWM 1100 <-> 1900, 2s interval)")

    def run_mission(self):
        # 1. Wait for MAVROS state connection
        while rclpy.ok() and not self.state_received:
            self.get_logger().info("Waiting for MAVROS state messages...", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok() and not self.current_state.connected:
            self.get_logger().info("Waiting for autopilot connection...", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Autopilot Connected! Starting RC Channel 9 servo toggle loop...")
        
        current_pwm = 1100
        
        while rclpy.ok():
            # Set RC Channel 9 override
            self.set_rc_channel(9, current_pwm)
            self.get_logger().info(f"[Servo CH9] Set PWM value: {current_pwm}")
            
            # Wait for 2.0 seconds while keeping ROS 2 callbacks spinning
            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time < 2.0):
                rclpy.spin_once(self, timeout_sec=0.1)

            # Toggle PWM between 1100 and 1900
            current_pwm = 1900 if current_pwm == 1100 else 1100

    def cleanup(self):
        """Release RC Channel 9 override on exit."""
        self.get_logger().info("Cleaning up Mission Servo Node... Releasing CH9 override.")
        self.set_rc_channel(9, 0)
        if hasattr(self, 'rc_pub') and self.rc_pub:
            try:
                self.publish_rc()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionServoNode()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected in Mission Servo Node.")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
