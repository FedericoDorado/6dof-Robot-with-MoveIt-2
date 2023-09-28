import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('centauri_6dof')
    urdf = os.path.join(package_dir,'models','centauri.urdf')
    
    default_rviz_config_path = os.path.join(package_dir,'rviz','centauri_config.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
    
    DeclareLaunchArgument(
	name='rviz_config_file',
	default_value=default_rviz_config_path,
	description='Full path to the RVIZ config file to use'),

     
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='JSP_gui',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
            
        Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file]
		),
        

    ])
