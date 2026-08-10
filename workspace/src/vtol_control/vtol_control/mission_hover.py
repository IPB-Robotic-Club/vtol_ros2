import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode

class MissionHoverNode(VtolBaseNode):
    def __init__(self):
        # Enable the background 10Hz RC publishing timer for overrides
        super().__init__('mission_hover_node', enable_rc_loop=True)
        self.get_logger().info("Mission Hover Node Started!")

    def run_mission(self):
        # 1. Takeoff using configured altitude and throttle
        if self.takeoff():
            # 2. Hover for exactly 10.0 seconds
            if self.hover(duration_seconds=10.0):
                # 3. Land gracefully and release overrides
                self.land()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionHoverNode()
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
