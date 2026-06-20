import rclpy
from vtol_control.vtol_base import VtolBaseNode
import time

class TestArmNode(VtolBaseNode):
    def __init__(self):
        super().__init__('test_arm_node')

        # Run logic in a periodic timer
        self.timer = self.create_timer(0.5, self.control_loop)
        self.phase = 'WAIT_CONNECTION'
        self.phase_start_time = 0.0

    def control_loop(self):
        if not self.state_received:
            self.get_logger().info("Waiting for MAVROS state messages...", throttle_duration_sec=2.0)
            return

        current_time = time.time()

        if self.phase == 'WAIT_CONNECTION':
            if self.current_state.connected:
                self.get_logger().info("Autopilot Connected! Changing mode to GUIDED...")
                self.change_mode("GUIDED")
                self.phase = 'SET_MODE_GUIDED'
                self.phase_start_time = current_time
            else:
                self.get_logger().info("Waiting for autopilot connection...", throttle_duration_sec=2.0)

        elif self.phase == 'SET_MODE_GUIDED':
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("Mode is GUIDED. Arming drone...")
                self.set_arm(True)
                self.phase = 'ARMING'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 5.0:
                self.get_logger().error("Failed to change mode to GUIDED. Retrying...")
                self.change_mode("GUIDED")
                self.phase_start_time = current_time

        elif self.phase == 'ARMING':
            if self.current_state.armed:
                self.get_logger().info("Drone successfully ARMED! Waiting 2 seconds...")
                self.phase = 'HOLD_ARMED'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 5.0:
                self.get_logger().error("Failed to ARM drone. Retrying...")
                self.set_arm(True)
                self.phase_start_time = current_time

        elif self.phase == 'HOLD_ARMED':
            elapsed = current_time - self.phase_start_time
            if elapsed >= 2.0:
                self.get_logger().info("2 seconds elapsed. Disarming drone...")
                self.set_arm(False)
                self.phase = 'DISARMING'
                self.phase_start_time = current_time

        elif self.phase == 'DISARMING':
            if not self.current_state.armed:
                self.get_logger().info("Drone successfully DISARMED! Returning to STABILIZE mode...")
                self.change_mode("STABILIZE")
                self.phase = 'SET_MODE_STABILIZE'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 5.0:
                self.get_logger().error("Failed to DISARM drone. Retrying...")
                self.set_arm(False)
                self.phase_start_time = current_time

        elif self.phase == 'SET_MODE_STABILIZE':
            if self.current_state.mode == "STABILIZE" or current_time - self.phase_start_time > 5.0:
                self.get_logger().info("Arming/Disarming test finished successfully. Shutting down node.")
                self.timer.cancel()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestArmNode()
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
