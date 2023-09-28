import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    package_dir = get_package_share_directory('centauri_6dof')
    urdf = os.path.join(package_dir,'models','3_centauri_2023.urdf')

    default_rviz_config_path = os.path.join(package_dir,'rviz','centauri_config3.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    pkg_share = FindPackageShare(package='centauri_6dof').find('centauri_6dof')
    gazebo_models_path = 'meshes'
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.15'
    spawn_yaw_val = '0.0'

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([

        DeclareLaunchArgument(
		name='rviz_config_file',
		default_value=default_rviz_config_path,
		description='Full path to the RVIZ config file to use'),

        Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file]),

         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),

#  Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-entity", "centauri6dof","-topic", "/robot_description",
		    '-x', spawn_x_val,
		    '-y', spawn_y_val,
		    '-z', spawn_z_val,
		    '-Y', spawn_yaw_val]),
#   Running the controllers in launch file
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'centauri_trajectory_controller'],
            output='screen'
         ),

        ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','centauri_hand_controller'],
        output='screen'
        ),


  ])