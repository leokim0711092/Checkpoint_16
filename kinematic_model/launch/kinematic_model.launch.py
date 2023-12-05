import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    kinematic_node = Node(
    package='kinematic_model',  
    executable='kinematic_model',  
    name='kinematic_model_node',
    output='screen',
    )

    wheel_vel_node = Node(
    package='wheel_velocities_publisher',  
    executable='wheel_velocities_publisher',  
    name='wheel_velocities_publisher_node',
    output='screen',
    )

    return LaunchDescription([
        kinematic_node,
        wheel_vel_node
    ])