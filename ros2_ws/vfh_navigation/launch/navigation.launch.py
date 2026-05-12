import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vfh_share  = get_package_share_directory('vfh_navigation')
    race_share = get_package_share_directory('turtlebot_gazebo_race')

    vfh_params   = os.path.join(vfh_share, 'config',    'vfh_params.yaml')
    wp_file      = os.path.join(vfh_share, 'waypoints', 'circuit_waypoints.yaml')
    rviz_config  = os.path.join(vfh_share, 'config',    'rviz_config.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(race_share, 'launch', 'create_multi_robot_race.launch.py')
        )
    )

    vfh_node = Node(
        package='vfh_navigation',
        executable='vfh_node',
        name='vfh_node',
        parameters=[vfh_params, {'waypoints_file': wp_file}],
        output='screen',
    )

    metrics_node = Node(
        package='vfh_navigation',
        executable='metrics_node',
        name='metrics_node',
        parameters=[vfh_params, {'waypoints_file': wp_file}],
        output='screen',
    )

    histogram_viz_node = Node(
        package='vfh_navigation',
        executable='histogram_viz',
        name='histogram_viz_node',
        parameters=[vfh_params],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Delay navigation nodes until Gazebo bridge has the /scan topic ready
    nav_delayed = TimerAction(
        period=8.0,
        actions=[vfh_node, metrics_node, histogram_viz_node, rviz_node],
    )

    return LaunchDescription([
        gazebo,
        nav_delayed,
    ])
