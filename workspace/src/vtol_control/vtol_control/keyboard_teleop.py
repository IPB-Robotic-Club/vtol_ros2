import rclpy
import sys
import os
import termios
import tty
import select
import time
import threading
from .drone_controller import DroneController

# ANSI terminal formatting
CLEAR_SCREEN = "\033[H\033[2J"
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_GREEN = "\033[32m"
COLOR_RED = "\033[31m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_CYAN = "\033[36m"
COLOR_MAGENTA = "\033[35m"

# Key list
msg = """
======================================================
  VTOL Drone Keyboard Controller & Telemetry HUD
======================================================
  Flight Controls:          Mode & Arming:
  ----------------          --------------
        i (Forward)         a : Arm
   j         l              d : Disarm
  (Left)  (Right)           t : Takeoff (5m)
        k (Backward)        g : Land
                            r : Return-To-Launch (RTL)
  w : Increase Alt (+z)     m : Toggle GUIDED / QLOITER
  s : Decrease Alt (-z)
  u : Yaw Left (CCW)        q : Quit / Safe Exit
  o : Yaw Right (CW)
  space : Hover / Stop
======================================================
"""

def get_key(settings):
    """Reads a single keypress from standard input in a non-blocking way."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def draw_hud(controller, target_vx, target_vy, target_vz, target_yaw_rate):
    """Draws a beautiful, real-time telemetry HUD in the terminal."""
    state = controller.current_state
    pose = controller.local_pose.pose
    vel = controller.local_velocity.twist
    gps = controller.global_position

    # Connection and Arming labels
    conn_str = f"{COLOR_GREEN}CONNECTED{COLOR_RESET}" if state.connected else f"{COLOR_RED}DISCONNECTED{COLOR_RESET}"
    arm_str = f"{COLOR_GREEN}ARMED{COLOR_RESET}" if state.armed else f"{COLOR_RED}DISARMED{COLOR_RESET}"
    guided_str = f"{COLOR_GREEN}GUIDED{COLOR_RESET}" if state.guided else f"{COLOR_RED}NON-GUIDED{COLOR_RESET}"

    # Flight Mode formatting
    mode_str = f"{COLOR_BLUE}{COLOR_BOLD}{state.mode}{COLOR_RESET}" if state.mode else "UNKNOWN"

    hud_content = []
    hud_content.append(CLEAR_SCREEN)
    hud_content.append(f"{COLOR_CYAN}{COLOR_BOLD}======================================================{COLOR_RESET}")
    hud_content.append(f"{COLOR_CYAN}{COLOR_BOLD}   VTOL FLIGHT TELEMETRY HUD (JAZZY / MAVROS)        {COLOR_RESET}")
    hud_content.append(f"{COLOR_CYAN}{COLOR_BOLD}======================================================{COLOR_RESET}")
    hud_content.append(f"  Autopilot Conn : {conn_str:<25} Mode : {mode_str}")
    hud_content.append(f"  Arm Status     : {arm_str:<25} Control: {guided_str}")
    hud_content.append(f"  Active Control : {COLOR_MAGENTA}{controller.control_mode}{COLOR_RESET}")
    hud_content.append(f"------------------------------------------------------")
    hud_content.append(f"  Local Position (meters):")
    hud_content.append(f"    X : {pose.position.x:7.2f}    Y : {pose.position.y:7.2f}    Z (Alt): {pose.position.z:7.2f}")
    hud_content.append(f"  Global Position (GPS):")
    hud_content.append(f"    Lat: {gps.latitude:10.6f}  Lon: {gps.longitude:10.6f}  Alt: {gps.altitude:7.2f}")
    hud_content.append(f"  Current Speeds (m/s):")
    hud_content.append(f"    Vx: {vel.linear.x:7.2f}    Vy: {vel.linear.y:7.2f}    Vz: {vel.linear.z:7.2f}")
    hud_content.append(f"------------------------------------------------------")
    hud_content.append(f"  Target Command Speeds:")
    hud_content.append(f"    Vx: {COLOR_YELLOW}{target_vx:5.1f}{COLOR_RESET} m/s | Vy: {COLOR_YELLOW}{target_vy:5.1f}{COLOR_RESET} m/s | Vz: {COLOR_YELLOW}{target_vz:5.1f}{COLOR_RESET} m/s | Yaw: {COLOR_YELLOW}{target_yaw_rate:5.1f}{COLOR_RESET} rad/s")
    hud_content.append(f"{COLOR_CYAN}{COLOR_BOLD}======================================================{COLOR_RESET}")
    hud_content.append(msg)
    
    # Print HUD
    sys.stdout.write("\n".join(hud_content))
    sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    
    # Save original terminal settings for non-blocking keys
    settings = termios.tcgetattr(sys.stdin)
    
    # Initialize our DroneController node
    controller = DroneController(node_name='keyboard_teleop_controller')
    
    # Spin ROS 2 callbacks in a background daemon thread using Executor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Request telemetry data stream rates (position, status, etc.) at 10Hz
    controller.get_logger().info("Requesting MAVROS stream rates at 10Hz...")
    controller.set_stream_rate(0, 10)  # Stream ID 0 = STREAM_ALL
    time.sleep(1.0)

    # Initial velocities
    vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
    
    # Step sizes
    v_step = 0.5        # m/s step
    yaw_step = 0.2      # rad/s step
    
    print("Initializing keyboard controls. Standby...")
    time.sleep(1.0)
    
    try:
        while rclpy.ok():
            draw_hud(controller, vx, vy, vz, yaw_rate)
            
            key = get_key(settings)
            
            if key == 'q':
                break
            
            # Arm / Disarm
            elif key == 'a':
                controller.arm(True)
            elif key == 'd':
                controller.arm(False)
            
            # Takeoff / Land / RTL
            elif key == 't':
                # Initiate a guided takeoff to 5m
                controller.takeoff(5.0)
            elif key == 'g':
                # Command landing - ArduPilot uses QLAND for VTOL, LAND for copter/plane
                # We will request LAND, but you can also use QLAND
                controller.set_mode("LAND")
            elif key == 'r':
                # RTL
                controller.set_mode("RTL")
                
            # Mode Toggle
            elif key == 'm':
                # Cycle between GUIDED and QLOITER for VTOL testing
                current = controller.current_state.mode
                next_mode = "QLOITER" if current == "GUIDED" else "GUIDED"
                controller.set_mode(next_mode)

            # Movement (Velocity Vectoring)
            elif key == 'i':
                vx += v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'k':
                vx -= v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'j':
                vy -= v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'l':
                vy += v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'w':
                vz += v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 's':
                vz -= v_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'u':
                yaw_rate += yaw_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
            elif key == 'o':
                yaw_rate -= yaw_step
                controller.set_velocity(vx, vy, vz, yaw_rate)
                
            # Space to hover / stop velocity
            elif key == ' ':
                vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
                controller.set_velocity(vx, vy, vz, yaw_rate)
                
            # Sleep slightly to control interface refresh rate
            time.sleep(0.05)
            
    except Exception as e:
        print(f"Error occurred in teleop: {e}")
        
    finally:
        # Reset terminal settings and cleanup
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        controller.get_logger().info("Shutting down keyboard controller...")
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
