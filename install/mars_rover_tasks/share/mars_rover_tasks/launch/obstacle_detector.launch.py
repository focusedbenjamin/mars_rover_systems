from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='mars_rover_tasks',
            executable='subscriber_obstacle_detector_executable',
            name='obstacle_detector_node',
            output='screen'
        )
    
    ])