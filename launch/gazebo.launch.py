from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.15'
    spawn_yaw_val = '0.0'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', 
            default_value=['/home/uao/ros2_ws/src/centauri_6dof/worlds/basic.world'],
            description='Path to Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-entity', 'centauri_robot',
                '-file', LaunchConfiguration('world'),
                '-x', spawn_x_val,
                '-y', spawn_y_val,
                '-z', spawn_z_val,
                '-Y', spawn_yaw_val
            ],
            output='screen',
        )
    ])