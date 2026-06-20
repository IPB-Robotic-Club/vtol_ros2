import rclpy
from vtol_control.vtol_base import VtolBaseNode

class VehicleStatusPrinter(VtolBaseNode):
    def __init__(self):
        super().__init__('vehicle_status_printer')

        # Print Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Clear screen once at the beginning
        print("\033[2J\033[H", end="", flush=True)

    def timer_callback(self):
        # Move cursor to top-left instead of full clear to prevent flickering
        print("\033[H", end="")

        print(f"==========================================\033[K")
        print(f"         VEHICLE STATUS PRINTER           \033[K")
        print(f"==========================================\033[K")

        # 1. Connection Status
        if self.current_state.connected:
            conn_str = "CONNECTED"
        else:
            conn_str = "DISCONNECTED"
        print(f" Autopilot Conn : {conn_str}\033[K")

        # 2. Arm Status
        arm_str = "ARMED" if self.current_state.armed else "DISARMED"
        print(f" Arm Status     : {arm_str}\033[K")

        # 3. Flight Mode
        mode_str = self.current_state.mode if self.current_state.mode else "UNKNOWN"
        print(f" Flight Mode    : {mode_str}\033[K")

        # 4. Battery
        if self.has_battery:
            volts = self.current_battery.voltage
            percentage = self.current_battery.percentage * 100.0
            bat_str = f"{volts:.2f}V ({percentage:.1f}%)"
        else:
            bat_str = "WAITING FOR DATA..."
        print(f" Battery        : {bat_str}\033[K")

        print(f"------------------------------------------\033[K")
        print(f" Local Position:\033[K")

        # 5. Local Position (XYZ)
        if self.has_pose:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            z = self.current_pose.pose.position.z
            pos_str = f"X: {x:7.2f} | Y: {y:7.2f} | Z (Alt): {z:7.2f}"
        else:
            pos_str = "  WAITING FOR LOCAL POSE..."
        print(f"  {pos_str}\033[K")
        print(f"==========================================\033[K", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusPrinter()
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
