import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vfh_share  = get_package_share_directory('vfh_navigation')
    race_share = get_package_share_directory('turtlebot_gazebo_race')

    vfh_params   = os.path.join(vfh_share, 'config',    'vfh_params.yaml')
    wp_file      = os.path.join(vfh_share, 'waypoints', 'circuit_waypoints.yaml')
    rviz_config  = os.path.join(vfh_share, 'config',    'rviz_config.rviz')

    # vfh_share = .../install/vfh_navigation/share/vfh_navigation → 5 niveles arriba = raíz del workspace
    logs_dir = os.path.realpath(
        os.path.join(vfh_share, '..', '..', '..', '..', '..', 'logs')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(race_share, 'launch', 'create_multi_robot_race.launch.py')
        )
    )

    vfh_node = Node(
        package='vfh_navigation',
        executable='vfh_node',
        name='vfh_node',
        parameters=[vfh_params, {'waypoints_file': wp_file, 'startup_delay': 5.0}],
        output='screen',
    )

    metrics_node = Node(
        package='vfh_navigation',
        executable='metrics_node',
        name='metrics_node',
        parameters=[vfh_params, {'waypoints_file': wp_file, 'logs_dir': logs_dir}],
        output='screen',
    )

    histogram_viz_node = Node(
        package='vfh_navigation',
        executable='histogram_viz',
        name='histogram_viz_node',
        parameters=[vfh_params],
        output='screen',
    )

    histogram_mpl_node = Node(
        package='vfh_navigation',
        executable='histogram_mpl',
        name='histogram_matplotlib_node',
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

    # Step 1: GUI windows start after Gazebo Sim bridge is ready (~8s)
    gui_delayed = TimerAction(
        period=8.0,
        actions=[rviz_node, histogram_mpl_node],
    )

    # Step 2: poll until all 3 windows are visible, position them, then exit
    # Layout (1920x1080): Gazebo Sim left half | RViz top-right | Matplotlib bottom-right
    arrange_process = ExecuteProcess(
        cmd=['bash', '-c',
             'until wmctrl -l | grep -qi "gazebo";       do sleep 0.5; done; '
             'until wmctrl -l | grep -qi "rviz";         do sleep 0.5; done; '
             'until wmctrl -l | grep -qi "VFH Histogram"; do sleep 0.5; done; '
             'sleep 1; '
             'wmctrl -r "Gazebo Sim"        -t 1; '
             'wmctrl -r "RViz"          -t 1; '
             'wmctrl -r "VFH Histogram" -t 1; '
             'wmctrl -r "Gazebo Sim"        -e 0,0,0,960,1080; '
             'wmctrl -r "RViz"          -e 0,960,0,960,540; '
             'wmctrl -r "VFH Histogram" -e 0,960,540,960,540; '
             'wmctrl -s 1'],
        output='log',
    )

    # Step 3: navigation nodes start only after windows are arranged
    nav_after_arrange = RegisterEventHandler(
        OnProcessExit(
            target_action=arrange_process,
            on_exit=[vfh_node, metrics_node, histogram_viz_node],
        )
    )

    return LaunchDescription([
        gazebo,
        gui_delayed,
        arrange_process,
        nav_after_arrange,
    ])
