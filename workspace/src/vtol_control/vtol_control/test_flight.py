import rclpy
import time
import sys
import threading
import math
from .drone_controller import DroneController

# Terminal formatting
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_CYAN = "\033[36m"
COLOR_RED = "\033[31m"

class TestFlightMission:
    def __init__(self, controller: DroneController):
        self.node = controller
        self.rate = 1.0  # Execution step check rate (Hz)

    def log(self, text, level="INFO"):
        prefix = f"{COLOR_BOLD}[TEST FLIGHT]{COLOR_RESET}"
        if level == "SUCCESS":
            print(f"{prefix} {COLOR_GREEN}{text}{COLOR_RESET}")
        elif level == "WARN":
            print(f"{prefix} {COLOR_YELLOW}WARNING: {text}{COLOR_RESET}")
        elif level == "ERROR":
            print(f"{prefix} {COLOR_RED}ERROR: {text}{COLOR_RESET}")
        else:
            print(f"{prefix} {COLOR_BLUE}{text}{COLOR_RESET}")

    def execute(self):
        self.log("Starting automated test flight mission...", "INFO")

        # Step 1: Wait for connection to autopilot
        self.log("Waiting for FCU connection...", "INFO")
        while rclpy.ok() and not self.node.current_state.connected:
            time.sleep(0.5)
        self.log("FCU connected!", "SUCCESS")

        # Request telemetry data stream rates (position, status, etc.) at 10Hz
        self.log("Requesting MAVROS stream rates at 10Hz...", "INFO")
        self.node.set_stream_rate(0, 10)  # Stream ID 0 = STREAM_ALL
        time.sleep(1.5)

        # Step 2: Set Mode to GUIDED
        # Note: Setpoints must be streaming first, which our controller does at 20Hz.
        self.log("Requesting GUIDED flight mode...", "INFO")
        mode_future = self.node.set_mode("GUIDED")

        # Wait for service call to complete (simplistic polling check)
        success = False
        for _ in range(10):
            if mode_future.done():
                response = mode_future.result()
                success = response.mode_sent
                break
            time.sleep(0.5)

        # Give autopilot a moment to transition and check telemetry
        time.sleep(1.0)
        if self.node.current_state.mode != "GUIDED":
            self.log(f"Mode transition failed. Current mode is {self.node.current_state.mode}. Retrying set_mode...", "WARN")
            self.node.set_mode("GUIDED")
            time.sleep(2.0)

        self.log(f"Current mode: {self.node.current_state.mode}", "SUCCESS")

        # Step 3: Arm the drone
        self.log("Sending Arming command...", "INFO")
        arm_future = self.node.arm(True)

        # Wait for arming to take effect
        for _ in range(10):
            if self.node.current_state.armed:
                break
            time.sleep(0.5)

        if not self.node.current_state.armed:
            self.log("Failed to arm drone! Exiting.", "ERROR")
            return
        self.log("Drone ARMED and ready!", "SUCCESS")

        # Step 4: Takeoff
        takeoff_alt = 5.0  # meters
        self.log(f"Initiating takeoff to {takeoff_alt} meters...", "INFO")

        takeoff_future = self.node.takeoff(takeoff_alt)

        # Wait for standard takeoff command response
        success = False
        for _ in range(10):
            if takeoff_future.done():
                response = takeoff_future.result()
                success = response.success
                self.log(f"Standard takeoff response: success={response.success}, result={response.result}", "INFO")
                break
            time.sleep(0.2)

        if not success:
            self.log("Standard takeoff rejected. Attempting MAV_CMD_NAV_VTOL_TAKEOFF...", "WARN")
            vtol_future = self.node.vtol_takeoff(takeoff_alt)
            for _ in range(10):
                if vtol_future.done():
                    response = vtol_future.result()
                    success = response.success
                    self.log(f"VTOL takeoff response: success={response.success}, result={response.result}", "INFO")
                    break
                time.sleep(0.2)

        if not success:
            self.log("Both takeoff commands rejected by autopilot! Aborting.", "ERROR")
            return

        # Monitor altitude until we reach target
        self.log("Ascending...", "INFO")
        while rclpy.ok():
            curr_alt = self.node.local_pose.pose.position.z
            self.log(f"Current Altitude: {curr_alt:4.2f} / {takeoff_alt:.1f} m")
            if abs(curr_alt - takeoff_alt) < 0.4:
                break
            # Failsafe check: if disarmed or disconnected, abort
            if not self.node.current_state.armed:
                self.log("Drone disarmed during takeoff! Aborting.", "ERROR")
                return
            time.sleep(1.0)

        self.log("Takeoff altitude reached!", "SUCCESS")
        time.sleep(2.0)

        # Step 5: Fly a Square pattern
        # Define local waypoints (relative to takeoff point)
        # Format: (x, y, z, description)
        waypoints = [
            (5.0, 0.0, takeoff_alt, "Waypoint 1 (North 5m)"),
            (5.0, 5.0, takeoff_alt, "Waypoint 2 (North-East 5m)"),
            (0.0, 5.0, takeoff_alt, "Waypoint 3 (East 5m)"),
            (0.0, 0.0, takeoff_alt, "Waypoint 4 (Return to Center)"),
        ]

        for x, y, z, desc in waypoints:
            self.log(f"Flying to {desc} at [{x:.1f}, {y:.1f}, {z:.1f}]...", "INFO")
            self.node.set_position(x, y, z)

            # Wait until we reach the waypoint
            while rclpy.ok():
                dist = self.node.get_distance_to_target()
                self.log(f"Distance to target: {dist:4.2f} meters")

                # Check if waypoint is reached
                if dist < 0.5:
                    self.log(f"Reached {desc}!", "SUCCESS")
                    time.sleep(2.0)  # Hover for 2 seconds at the waypoint
                    break

                # Safety checks
                if not self.node.current_state.armed:
                    self.log("Drone disarmed during navigation! Aborting.", "ERROR")
                    return
                if self.node.current_state.mode != "GUIDED":
                    self.log(f"Flight mode changed to {self.node.current_state.mode}! Aborting mission.", "ERROR")
                    return

                time.sleep(0.8)

        # Step 6: Return to Launch & Land
        self.log("Waypoints complete. Command RTL (Return to Launch)...", "INFO")
        self.node.set_mode("RTL")

        # Monitor altitude decay until landed and disarmed
        self.log("Returning and landing. Monitoring altitude...", "INFO")
        while rclpy.ok() and self.node.current_state.armed:
            curr_alt = self.node.local_pose.pose.position.z
            self.log(f"Landing Alt: {curr_alt:4.2f} m | Mode: {self.node.current_state.mode}")
            time.sleep(1.5)

        self.log("Test flight mission completed successfully!", "SUCCESS")


def main(args=None):
    rclpy.init(args=args)

    # Initialize the drone controller node
    controller = DroneController(node_name='test_flight_mission_node')

    # Use SingleThreadedExecutor for clean thread management
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(controller)

    # Spin ROS 2 in a background daemon thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run the mission flow
    mission = TestFlightMission(controller)
    try:
        mission.execute()
    except KeyboardInterrupt:
        mission.log("Mission interrupted by keyboard command. Commanding Land...", "WARN")
        # Attempt emergency landing
        controller.set_mode("LAND")
    finally:
        # Cleanup properly in order
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
