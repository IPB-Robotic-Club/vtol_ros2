from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Detect the gateway IP of WSL2 to connect to local host's Mission Planner SITL
    win_ip = os.environ.get('WIN_IP', '127.0.0.1')
    default_fcu_url = f"tcp://{win_ip}:5762"

    # Declare fcu_url launch argument, allowing the user to override it on command line
    fcu_url_launch_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value=default_fcu_url,
        description='MAVLink FCU URL (e.g. tcp://172.19.80.1:5762 or /dev/ttyACM0:921600)'
    )

    # Launch MAVROS node - output redirected to log to prevent polluting the terminal output
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='log',
        arguments=['--ros-args', '--log-level', 'FATAL'],
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
        }]
    )

    # Launch vehicle status printer node
    vehicle_status_node = Node(
        package='vtol_control',
        executable='vehicle_status',
        name='vehicle_status_printer',
        output='screen'
    )

    return LaunchDescription([
        fcu_url_launch_arg,
        mavros_node,
        vehicle_status_node
    ])
