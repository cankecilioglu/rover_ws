import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('rover_bringup')
    pkg_mapping = FindPackageShare('rover_mapping')

    declare_world = DeclareLaunchArgument(
        'world', default_value='sensor_world.sdf',
        description='rover_bringup/worlds icindeki world dosyasi')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='RViz ac/kapa')
    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value=os.path.expanduser('~/rover_maps'),
        description='Harita kayit dizini')

    # 1) Sim + robot + controller + bridge'ler (mevcut launch)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'rover_gazebo.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world')}.items())

    # 2) Heatmap node (sim ayaga kalksin diye gecikmeli)
    heatmap = TimerAction(period=4.0, actions=[
        Node(package='rover_mapping', executable='terrain_heatmap_node',
             name='terrain_heatmap_node', output='screen')])

    # 3) Autosave node (Ctrl-C'de + /save_map servisinde kaydeder)
    autosave = TimerAction(period=4.0, actions=[
        Node(package='rover_mapping', executable='map_autosave_node',
             name='map_autosave_node', output='screen',
             parameters=[{'map_dir': LaunchConfiguration('map_dir')}])])

    # 4) RViz (heatmap config ile)
    rviz = TimerAction(period=6.0, actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution(
                 [pkg_mapping, 'config', 'rover_heatmap.rviz'])],
             output='log',
             condition=IfCondition(LaunchConfiguration('rviz')))])

    return LaunchDescription([
        declare_world, declare_rviz, declare_map_dir,
        gazebo, heatmap, autosave, rviz])