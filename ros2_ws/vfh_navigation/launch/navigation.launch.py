import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vfh_share  = get_package_share_directory('vfh_navigation')
    race_share = get_package_share_directory('turtlebot_gazebo_race')

    vfh_params   = os.path.join(vfh_share, 'config',    'vfh_params.yaml')
    slam_params  = os.path.join(vfh_share, 'config',    'slam_params.yaml')
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

    slam_share = get_package_share_directory('slam_toolbox')
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time':     'true',
        }.items(),
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

    path_recorder_node = Node(
        package='vfh_navigation',
        executable='path_recorder',
        name='path_recorder_node',
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

    # Step 1: GUI windows + SLAM start after Gazebo Sim bridge is ready (~8s)
    gui_delayed = TimerAction(
        period=8.0,
        actions=[rviz_node, histogram_mpl_node, slam_node],
    )

    # Step 2: navigation nodes start at 13s — after Gazebo (8s) + robot spawning (~5s).
    # Independent of window arrangement so a slow matplotlib import never blocks driving.
    nav_delayed = TimerAction(
        period=13.0,
        actions=[vfh_node, metrics_node, path_recorder_node, histogram_viz_node],
    )

    # Step 3: arrange windows in parallel — best-effort, does not block anything.
    # Layout (1920x1080): Gazebo Sim left half | RViz top-right | Matplotlib bottom-right
    arrange_process = ExecuteProcess(
        cmd=['bash', '-c',
             'until wmctrl -l | grep -qi "gazebo";        do sleep 0.5; done; '
             'until wmctrl -l | grep -qi "rviz";          do sleep 0.5; done; '
             'until wmctrl -l | grep -qi "VFH Histogram"; do sleep 0.5; done; '
             'sleep 1; '
             'wmctrl -r "Gazebo Sim"    -t 1; '
             'wmctrl -r "RViz"          -t 1; '
             'wmctrl -r "VFH Histogram" -t 1; '
             'wmctrl -r "Gazebo Sim"    -e 0,0,0,960,1080; '
             'wmctrl -r "RViz"          -e 0,960,0,960,540; '
             'wmctrl -r "VFH Histogram" -e 0,960,540,960,540; '
             'wmctrl -s 1'],
        output='log',
    )

    detector_node = Node(
        package='detector_pkg',
        executable='detector_node',
        name='detector_node',
        output='screen'
    )

    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/detections/image'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        gui_delayed,
        nav_delayed,
        arrange_process,
        detector_node,
        rqt_node,
    ])
