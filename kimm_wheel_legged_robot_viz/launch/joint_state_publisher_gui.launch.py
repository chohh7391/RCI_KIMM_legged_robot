from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration(
        'use_sim_time', default='false'
    )

    urdf_path = PathJoinSubstitution([
        get_package_share_directory('kimm_wheel_legged_robot_description'),
        'urdf', 'kimm_wheel_legged_robot_walk.urdf'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_path])},
            {'use_sim_time': use_sim_time},
        ],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config = PathJoinSubstitution([
        get_package_share_directory('kimm_wheel_legged_robot_viz'),
        'rviz', 'joint_state_publisher_gui.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node,
    ])