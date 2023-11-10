from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topics_quiz',
            executable='agv_sim_oa_sub_pub_node',
            output='screen'
        ),
    
    ])
