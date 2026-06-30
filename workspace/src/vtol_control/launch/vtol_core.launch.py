from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Retrieve the configuration path for the MAVLink interface
    package_share_dir = get_package_share_directory('vtol_control')
    
    # Import and read fcu_url using config_reader
    try:
        from vtol_control.config_reader import get_fcu_url
        fcu_url = get_fcu_url()
    except ImportError:
        import sys
        # Fallback: add source directory to sys.path if not sourced
        src_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', 'vtol_control'))
        if src_path not in sys.path:
            sys.path.append(src_path)
        try:
            from vtol_control.config_reader import get_fcu_url
            fcu_url = get_fcu_url()
        except Exception as e:
            print(f"Warning: Failed to import config_reader: {e}. Using default URL.")
            fcu_url = 'tcp://127.0.0.1:5762'

    # Path to FastDDS configuration file to prevent WSL2 shared memory communication lockups
    src_fastdds_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', 'vtol_control', 'config', 'fastdds.xml'))
    installed_fastdds_path = os.path.join(package_share_dir, 'config', 'fastdds.xml')
    fastdds_path = src_fastdds_path if os.path.exists(src_fastdds_path) else installed_fastdds_path

    print(f"======================================================")
    print(f" VTOL Core Launching with FCU URL: {fcu_url}")
    print(f" FastDDS Configuration File: {fastdds_path}")
    print(f"======================================================")

    # Declare Launch Argument for HUD visualization
    show_hud_arg = DeclareLaunchArgument(
        'show_hud',
        default_value='false',
        description='Whether to launch the vehicle status HUD printer node'
    )
    show_hud = LaunchConfiguration('show_hud')

    # Launch MAVROS node - output to screen with default INFO log level to diagnose connection problems
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'system_id': 255,
        }]
    )

    # Launch the VTOL core node (monitoring and failsafes)
    vtol_core_node = Node(
        package='vtol_control',
        executable='vtol_core',
        name='vtol_core_node',
        output='screen'  # Print logs directly to console for visibility
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
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_path),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        show_hud_arg,
        mavros_node,
        vtol_core_node,
        vehicle_status_node
    ])
