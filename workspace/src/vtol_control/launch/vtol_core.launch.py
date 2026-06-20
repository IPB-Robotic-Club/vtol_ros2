from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Retrieve the configuration path for the MAVLink interface
    package_share_dir = get_package_share_directory('vtol_control')
    
    # Try to find the source directory config file first (so user changes in src/ apply instantly without colcon build)
    # package_share_dir: workspace/install/vtol_control/share/vtol_control
    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', 'vtol_control', 'config', 'fcu_url.txt'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'fcu_url.txt')
    
    if os.path.exists(src_config_path):
        config_file_path = src_config_path
        print(f"Loading FCU URL from source config: {config_file_path}")
    else:
        config_file_path = installed_config_path
        print(f"Loading FCU URL from installed config: {installed_config_path}")

    # Default fallback
    fcu_url = 'tcp://127.0.0.1:5762'

    # Read fcu_url from the config file if it exists
    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip empty or commented lines
                    if line and not line.startswith('#'):
                        fcu_url = line
                        break
        except Exception as e:
            print(f"Warning: Failed to read config file {config_file_path}: {e}. Falling back to default URL.")

    print(f"======================================================")
    print(f" VTOL Core Launching with FCU URL: {fcu_url}")
    print(f"======================================================")

    # Declare Launch Argument for HUD visualization
    show_hud_arg = DeclareLaunchArgument(
        'show_hud',
        default_value='false',
        description='Whether to launch the vehicle status HUD printer node'
    )
    show_hud = LaunchConfiguration('show_hud')

    # Launch MAVROS node - output redirected to log to prevent polluting the terminal output
    # Log level is set to FATAL to suppress warning/info noise from MAVROS in the console
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='log',
        arguments=['--ros-args', '--log-level', 'FATAL'],
        parameters=[{
            'fcu_url': fcu_url,
        }]
    )

    # Launch the VTOL core node (monitoring and failsafes)
    vtol_core_node = Node(
        package='vtol_control',
        executable='vtol_core',
        name='vtol_core_node',
        output='log'  # Redirect logs to file to keep console clean for HUD if shown
    )

    # Launch the vehicle status HUD printer conditionally
    vehicle_status_node = Node(
        package='vtol_control',
        executable='vehicle_status',
        name='vehicle_status_printer',
        output='screen',
        condition=IfCondition(show_hud)
    )

    return LaunchDescription([
        show_hud_arg,
        mavros_node,
        vtol_core_node,
        vehicle_status_node
    ])
