import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode
import time

class MissionManeuverNode(VtolBaseNode):
    def __init__(self):
        # Enable the background 10Hz RC publishing timer for overrides
        super().__init__('mission_maneuver_node', enable_rc_loop=True)
        self.get_logger().info("Mission Maneuver Node Started!")

    def execute_maneuver(self, action_name, roll=1500, pitch=1500, yaw=1500, throttle=1500, duration=2.0):
        self.get_logger().info(f"Executing Maneuver: {action_name} (Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}, Throttle: {throttle}) for {duration}s...")
        
        self.rc_channels[0] = roll
        self.rc_channels[1] = pitch
        self.rc_channels[2] = throttle
        self.rc_channels[3] = yaw
        
        if not self.rc_timer:
            self.publish_rc()

        start_time = time.time()
        while rclpy.ok():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Watchdog check: connection
            if self.state_received and not self.current_state.connected:
                self.get_logger().error("Autopilot disconnected during maneuver! Aborting.")
                self.abort_flight()
                return False

            # Watchdog check: manual flight mode override by pilot/GCS
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(f"Manual override detected! Flight mode changed to {self.current_state.mode}. Aborting mission.")
                self.abort_flight()
                return False
                
            if elapsed >= duration:
                # Return to neutral controls
                self.rc_channels[0] = 1500
                self.rc_channels[1] = 1500
                self.rc_channels[2] = 1500
                self.rc_channels[3] = 1500
                if not self.rc_timer:
                    self.publish_rc()
                return True
                
            self.get_logger().info(f"Maneuvering... elapsed: {elapsed:.1f}s / {duration}s", throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return False

    def run_mission(self):
        # 1. Takeoff using configured altitude and throttle
        if not self.takeoff():
            return

        # 2. Hover to stabilize
        if not self.hover(duration_seconds=3.0):
            return

        # 3. Roll demonstration (Right, then Left)
        # Roll Right (RC1 = 1600)
        if not self.execute_maneuver("ROLL RIGHT", roll=1600, duration=1.5):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return
        # Roll Left (RC1 = 1400)
        if not self.execute_maneuver("ROLL LEFT", roll=1400, duration=1.5):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return

        # 4. Pitch demonstration (Forward, then Backward)
        # Pitch Forward (RC2 = 1400)
        if not self.execute_maneuver("PITCH FORWARD", pitch=1400, duration=1.5):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return
        # Pitch Backward (RC2 = 1600)
        if not self.execute_maneuver("PITCH BACKWARD", pitch=1600, duration=1.5):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return

        # 5. Yaw demonstration (Yaw Right, then Yaw Left)
        # Yaw Right (RC4 = 1650)
        if not self.execute_maneuver("YAW RIGHT", yaw=1650, duration=2.0):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return
        # Yaw Left (RC4 = 1350)
        if not self.execute_maneuver("YAW LEFT", yaw=1350, duration=2.0):
            return
        # Stabilize
        if not self.hover(duration_seconds=2.0):
            return

        # 6. Land gracefully
        self.land()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionManeuverNode()
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
