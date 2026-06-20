import rclpy
from rclpy.signals import SignalHandlerOptions
from vtol_control.vtol_base import VtolBaseNode
import time

class MissionHoverNode(VtolBaseNode):
    def __init__(self):
        super().__init__('mission_hover_node')

        # Background RC publisher timer (10 Hz)
        self.rc_timer = self.create_timer(0.1, self.publish_rc)

        # Control loop state machine timer (2 Hz)
        self.control_timer = self.create_timer(0.5, self.control_loop)

        # State machine variables
        self.phase = 'WAIT_CONNECTION'
        self.phase_start_time = 0.0
        self.safety_aborted = False

        self.get_logger().info("Mission Hover Node Started!")

    def on_state(self, msg):
        # Safety Watchdog: If we are in TAKEOFF or HOVER phase (active RC override)
        # and the pilot changes the flight mode manually to anything else (e.g. STABILIZE),
        # we must instantly abort, release overrides, and stop sending commands.
        if self.phase in ['TAKEOFF', 'HOVER']:
            if msg.mode not in ['LOITER', 'CMODE(5)']:
                self.get_logger().warn(f"Manual override detected! Flight mode changed to {msg.mode}. Aborting mission.")
                self.abort_mission()

    def control_loop(self):
        if self.safety_aborted:
            return

        if not self.state_received:
            self.get_logger().info("Waiting for MAVROS state messages...", throttle_duration_sec=2.0)
            return

        current_time = time.time()

        if self.phase == 'WAIT_CONNECTION':
            if self.current_state.connected:
                self.get_logger().info("Autopilot Connected! Changing mode to LOITER...")
                self.change_mode("LOITER")
                self.phase = 'SET_MODE_LOITER'
                self.phase_start_time = current_time
            else:
                self.get_logger().info("Waiting for autopilot connection...", throttle_duration_sec=2.0)

        elif self.phase == 'SET_MODE_LOITER':
            if self.current_state.mode in ["LOITER", "CMODE(5)"]:
                self.get_logger().info("Mode is LOITER. Arming drone...")
                self.set_arm(True)
                self.phase = 'ARMING'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 5.0:
                self.get_logger().error("Failed to change mode to LOITER. Retrying...")
                self.change_mode("LOITER")
                self.phase_start_time = current_time

        elif self.phase == 'ARMING':
            if self.current_state.armed:
                self.get_logger().info("Drone successfully ARMED! Starting Takeoff...")
                # Start takeoff throttle: Roll=1500, Pitch=1500, Throttle=1700 (climb), Yaw=1500
                self.rc_channels[0] = 1500 # Roll
                self.rc_channels[1] = 1500 # Pitch
                self.rc_channels[2] = 1700 # Throttle (Climb)
                self.rc_channels[3] = 1500 # Yaw
                self.phase = 'TAKEOFF'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 5.0:
                self.get_logger().error("Failed to ARM drone. Retrying...")
                self.set_arm(True)
                self.phase_start_time = current_time

        elif self.phase == 'TAKEOFF':
            current_alt = self.current_pose.pose.position.z if self.has_pose else 0.0
            elapsed = current_time - self.phase_start_time
            if current_alt >= 2.0:
                self.get_logger().info(f"Target altitude reached ({current_alt:.2f}m >= 2.0m). Entering Hover phase...")
                # Set throttle to neutral 1500 to hover
                self.rc_channels[2] = 1500
                self.phase = 'HOVER'
                self.phase_start_time = current_time
            elif elapsed > 20.0:
                self.get_logger().error(f"Takeoff timeout! Failed to reach 2.0m in 20 seconds. Current altitude: {current_alt:.2f}m. Aborting.")
                self.abort_mission()
            else:
                self.get_logger().info(f"Climbing... alt: {current_alt:.2f}m/2.0m, throttle: 1700, elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)

        elif self.phase == 'HOVER':
            elapsed = current_time - self.phase_start_time
            if elapsed >= 5.0:
                self.get_logger().info("Hover complete (5.0s). Switching mode to LAND for graceful landing...")
                # Change mode to LAND to descend gracefully
                self.change_mode("LAND")
                # Immediately release overrides (send 0s) so autopilot handles the landing autonomously
                self.rc_channels = [0] * 18
                self.phase = 'LANDING'
                self.phase_start_time = current_time
            else:
                self.get_logger().info(f"Hovering... throttle: 1500, elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)

        elif self.phase == 'LANDING':
            # Wait until disarmed
            if not self.current_state.armed:
                self.get_logger().info("Drone successfully DISARMED on ground. Mission complete!")
                self.phase = 'FINISHED'
                self.phase_start_time = current_time
            elif current_time - self.phase_start_time > 15.0 and self.current_state.mode != "LAND":
                # Fallback check if mode change failed
                self.get_logger().error("LAND mode not active, retrying LAND command...")
                self.change_mode("LAND")
                self.phase_start_time = current_time

        elif self.phase == 'FINISHED':
            self.get_logger().info("Exiting mission node.")
            self.control_timer.cancel()
            self.rc_timer.cancel()
            rclpy.shutdown()

    def abort_mission(self):
        self.safety_aborted = True
        # Immediately set all overrides to 0 to return full control to physical RC
        self.rc_channels = [0] * 18
        # Publish it immediately to MAVROS
        self.publish_rc()
        self.get_logger().error("MISSION ABORTED: Released all RC overrides (set to 0).")
        # Cancel timers
        self.control_timer.cancel()
        self.rc_timer.cancel()
        rclpy.shutdown()

    def destroy_node(self):
        # Safety release on shutdown
        if rclpy.ok():
            try:
                self.rc_channels = [0] * 18
                self.publish_rc()
            except Exception as e:
                self.get_logger().warn(f"Failed to publish safety RC release: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt detected! Performing safety landing sequence...")
        if node.state_received and node.current_state.armed:
            node.change_mode("LAND")
            node.rc_channels = [0] * 18
            try:
                node.publish_rc()
            except Exception as e:
                node.get_logger().warn(f"Failed to publish safety RC release: {e}")
            # Spin briefly to allow the async mode change service call to be processed
            start_time = time.time()
            while rclpy.ok() and time.time() - start_time < 1.5:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
