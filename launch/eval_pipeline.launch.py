import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('agentic_racer')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {pkg_share}/track4.sdf'}.items()
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'kidney_track',
            '-file', os.path.join(get_package_share_directory('orbit_bot'), 'urdf', 'orbit_bot.urdf'),
            '-name', 'orbit_bot',
            '-x', '1.125',
            '-y', '-4.5',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel_test@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/bumper@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )
    
    autopilot = Node(
        package='agentic_racer',
        executable='autopilot_node',
        output='screen'
    )
    
    referee = Node(
        package='agentic_racer',
        executable='referee_node',
        name='referee_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge,
        autopilot,
        referee
    ])
