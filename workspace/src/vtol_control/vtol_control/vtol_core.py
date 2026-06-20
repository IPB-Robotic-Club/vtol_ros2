import rclpy
from vtol_control.vtol_base import VtolBaseNode
from mavros_msgs.srv import StreamRate
from mavros_msgs.msg import StatusText
import time

class VtolCore(VtolBaseNode):
    def __init__(self):
        super().__init__('vtol_core')

        # Timer for Failsafe Watchdog (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Service client for stream rate (not in base node)
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # Subscribe to MAVROS statustext to print autopilot messages in real-time
        self.statustext_sub = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.statustext_callback,
            self.qos_telemetry
        )

        # Watchdog and safety state variables
        self.stream_rate_requested = False
        self.mavros_heartbeat_ok = False
        self.autopilot_connected = False
        
        # Safety overrides
        self.mission_completed = False
        self.pilot_override_active = False

        self.get_logger().info('VTOL Core Node Started!')

    def on_state(self, msg):
        self.mavros_heartbeat_ok = True

        # Log connection status changes to keep the console informed
        if msg.connected != self.autopilot_connected:
            if msg.connected:
                self.get_logger().info("Autopilot Link Connected successfully!")
            else:
                self.get_logger().warn("Autopilot Link Connection lost!")

        self.autopilot_connected = msg.connected

        # Detect GCS or Pilot manual override to LAND mode
        if msg.mode in ["LAND", "CMODE(9)"] and not self.mission_completed:
            self.mission_completed = True
            self.pilot_override_active = True
            self.get_logger().info("GCS/Pilot manual override to LAND mode detected. Mission completed.")

    def request_stream_rate(self):
        if self.stream_rate_client.wait_for_service(timeout_sec=0.5):
            req = StreamRate.Request()
            req.stream_id = 0  # STREAM_ALL
            req.message_rate = 10
            req.on_off = True
            self.stream_rate_client.call_async(req)
            self.stream_rate_requested = True
            self.get_logger().info("Requested autopilot telemetry stream rate at 10Hz.")

    def trigger_failsafe_land(self):
        """Attempts to command LAND mode as a critical safety failsafe."""
        self.get_logger().error("FAILSAFE TRIGGERED: Ordering immediate LAND!")
        self.change_mode("LAND")

    def timer_callback(self):
        current_time = time.time()

        # Provide console feedback during startup connection phase
        if not self.state_received:
            self.get_logger().info("Waiting for MAVROS state messages (check if MAVROS node is running)...", throttle_duration_sec=3.0)
        elif not self.autopilot_connected:
            self.get_logger().info("MAVROS node is active. Waiting for autopilot link connection (check if SITL is running and fcu_url is correct)...", throttle_duration_sec=3.0)

        # 1. Watchdog: Check MAVROS Node Health (Heartbeat)
        # Only check and trigger failsafe if the drone is armed (flying)
        if self.current_state.armed:
            if self.last_state_time > 0.0:
                time_since_last_state = current_time - self.last_state_time
                if time_since_last_state > 8.0:
                    if self.mavros_heartbeat_ok:
                        self.get_logger().error("Watchdog: MAVROS Heartbeat Lost during flight! (No state messages for 8 seconds)")
                    self.mavros_heartbeat_ok = False
                    self.trigger_failsafe_land()
        else:
            # If not armed, keep resetting the watchdog health state silently
            if self.last_state_time > 0.0:
                time_since_last_state = current_time - self.last_state_time
                self.mavros_heartbeat_ok = (time_since_last_state <= 8.0)
            else:
                self.mavros_heartbeat_ok = False

        # 2. Watchdog: Check Autopilot Connection Health
        if self.mavros_heartbeat_ok and not self.autopilot_connected:
            # MAVROS is running, but autopilot connection lost during armed state
            if self.current_state.armed:
                self.get_logger().error("Watchdog: Autopilot Link Connection Lost during flight!")
                self.trigger_failsafe_land()

        # 3. Request telemetry stream rates once connected
        if self.autopilot_connected and not self.stream_rate_requested:
            self.request_stream_rate()

    def statustext_callback(self, msg):
        # Forward autopilot status and alert messages to the screen
        if msg.severity <= 3: # EMERGENCY, ALERT, CRITICAL, ERROR
            self.get_logger().error(f"[Autopilot Alert]: {msg.text}")
        elif msg.severity == 4: # WARNING
            self.get_logger().warn(f"[Autopilot Warning]: {msg.text}")
        else: # NOTICE, INFO, DEBUG
            self.get_logger().info(f"[Autopilot Status]: {msg.text}")

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
