import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode
import time

class TestTakeoffNode(VtolBaseNode):
    def __init__(self):
        # Enable the background 10Hz RC publishing timer for overrides
        super().__init__('test_takeoff_node', enable_rc_loop=True)
        self.get_logger().info("Test Takeoff Node Started! (Takeoff & Hover Continuous)")

    def run_mission(self):
        # 1. Takeoff using configured altitude and throttle from vtol_config.yaml
        if not self.takeoff():
            return

        # 2. Hover continuously until user cancels with Ctrl+C
        self.get_logger().info(
            "Takeoff berhasil! Drone sekarang melayang (hover) di udara.\n"
            "Tekan Ctrl+C kapan saja untuk mendaratkan drone secara otomatis (LAND)."
        )

        while rclpy.ok():
            # Watchdog check: connection
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected during hover! Aborting.")
                self.abort_flight()
                return

            # Watchdog check: manual flight mode override by pilot/GCS
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(
                    f"Intervensi manual terdeteksi! Flight mode berubah ke {self.current_state.mode}. Abort flight."
                )
                self.abort_flight()
                return

            # Maintain neutral RC channels
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = 1500
            self.rc_channels[3] = 1500

            alt = self.get_current_altitude()
            self.get_logger().info(f"[HOVERING] Altitude: {alt:.2f}m | Mode: {self.current_state.mode}", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TestTakeoffNode()
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
