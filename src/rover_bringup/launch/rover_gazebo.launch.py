from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_rover_description = FindPackageShare('rover_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    xacro_file = PathJoinSubstitution([
        pkg_rover_description, 'urdf', 'rover.urdf.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        ),
        'use_sim_time': True,
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_ros_gz_sim, '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-v 4 -r empty.sdf'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', '0.1',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn,
    ])