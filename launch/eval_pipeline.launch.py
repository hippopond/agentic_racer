import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Placeholder for the automated Eval Pipeline
    
    # 1. The F1Tenth Simulator (Headless Mode)
    # 2. The Autopilot (Pure Pursuit)
    
    # 3. The Referee Node
    referee = Node(
        package='agentic_racer',
        executable='referee_node',
        output='screen'
    )
    
    return LaunchDescription([
        referee
    ])
