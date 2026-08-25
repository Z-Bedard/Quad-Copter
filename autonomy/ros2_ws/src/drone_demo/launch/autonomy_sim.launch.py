from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_demo',
            executable='bridge_sim_node',
            name='bridge_sim_node'
        ),

        Node(
            package='drone_demo',
            executable='guidance_node',
            name='guidance_node'
        ),
    ])