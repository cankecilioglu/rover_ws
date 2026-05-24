from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_rover_description = FindPackageShare('rover_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_rover_bringup = FindPackageShare('rover_bringup')
    world_file = PathJoinSubstitution([pkg_rover_bringup, 'worlds', 'sensor_world.sdf'])    
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
        launch_arguments={'gz_args': ['-v 4 -r ', world_file]}.items()
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
            '-z', '0.5',
        ],
        output='screen',
    )
    gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            remappings=[
                ('/model/rover/odometry', '/odometry/ground_truth'),
            ],
            output='screen',
        )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
    )
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_localization'),
                'config',
                'ekf_local.yaml'
            ])
        ],
    )
    ground_truth_tf = Node(
        package='rover_localization',
        executable='ground_truth_tf.py',
        name='ground_truth_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    gps_cov_relay = Node(
        package='rover_localization',
        executable='gps_covariance_relay.py',
        name='gps_covariance_relay',
        output='screen',
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_localization'),
                'config',
                'ekf_global.yaml'
            ])
        ],
        remappings=[('odometry/filtered', 'odometry/filtered_map')],
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_localization'),
                'config',
                'navsat_transform.yaml'
            ])
        ],
        remappings=[
            ('imu', '/imu'),
            ('gps/fix', '/gps/fix'),                 # ← relay'in çıktısı
            ('odometry/filtered', '/odometry/filtered_map'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # Sıralama: rover spawn olduktan sonra JSB, JSB başladıktan sonra diff_drive
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn,
        delay_jsb,
        delay_diff_drive,
        gz_bridge, 
        ekf_local,
        ground_truth_tf,
        gps_cov_relay,
        ekf_global,
        navsat_transform,
    ])