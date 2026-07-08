from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, SetEnvironmentVariable
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
    print(f" VTOL Autonomous Arming Test Launching with FCU: {fcu_url}")
    print(f" FastDDS Configuration File: {fastdds_path}")
    print(f"======================================================")

    # Launch MAVROS node - output redirected to log
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

    # Launch the VTOL core node - output redirected to log
    vtol_core_node = Node(
        package='vtol_control',
        executable='vtol_core',
        name='vtol_core_node',
        output='log'
    )

    # Launch the vehicle status HUD printer - output to screen
    vehicle_status_node = Node(
        package='vtol_control',
        executable='vehicle_status',
        name='vehicle_status_printer',
        output='screen'
    )

    # Launch the test_arm sequence. Trigger launch shutdown when this node exits.
    test_arm_node = Node(
        package='vtol_control',
        executable='test_arm',
        name='test_arm_node',
        output='screen',
        on_exit=Shutdown()
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_path),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        mavros_node,
        vtol_core_node,
        vehicle_status_node,
        test_arm_node
    ])
