import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Directorios
    pkg_share = get_package_share_directory('turtlebot_gazebo_race')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Archivo del Mundo y Modelo
    sdf_file = os.path.join(pkg_share, 'worlds', 'race.sdf')
    
    # Obtener modelo del robot de variable de entorno o argumento
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    
    # Ruta al URDF del Turtlebot3 oficial
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf'
    )

    # Ruta al modelo SDF (incluye plugins de movimiento para gz-sim)
    custom_model_sdf_file = os.path.join(
        pkg_share,
        'models',
        'turtlebot3_waffle_kinect',
        'model.sdf'
    )
    if os.path.exists(custom_model_sdf_file):
        model_sdf_file = custom_model_sdf_file
    else:
        model_sdf_file = os.path.join(
            pkg_turtlebot3_gazebo,
            'models',
            f'turtlebot3_{turtlebot3_model}',
            'model.sdf'
        )
        if not os.path.exists(model_sdf_file):
            model_sdf_file = os.path.join(
                pkg_turtlebot3_gazebo,
                'models',
                'turtlebot3_waffle',
                'model.sdf'
            )

    # Añadir ruta de modelos al resource path de Gazebo (igual que empty_world)
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # Servidor de Gazebo (sin GUI)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', sdf_file],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Cliente de Gazebo (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
    )

    # Publicar el Estado del Robot (Robot State Publisher)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', urdf_file])
        }]
    )

    # Publica /joint_states
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    # Spawnear el Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-file', model_sdf_file,
            '-x', '9.05',
            '-y', '9.00',
            '-z', '0.0',
            '-Y', '-3.11'
        ],
        output='screen'
    )

    # Bridge ROS 2 <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./scan.publisher.reliability': 'best_effort',
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/kinect/rgb@sensor_msgs/msg/Image[gz.msgs.Image',
            '/kinect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/kinect/depth@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/kinect/rgb', '/camera/rgb/image_raw'),
            ('/kinect/camera_info', '/camera/rgb/camera_info'),
            ('/kinect/depth', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # Publica CameraInfo de profundidad
    depth_camera_info = Node(
        package='turtlebot_gazebo_multiple',
        executable='depth_camera_info_publisher',
        name='depth_camera_info_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'depth_image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'width': 640,
            'height': 480,
            'hfov': 1.047,
            'frame_id': 'camera_rgb_optical_frame',
        }],
    )

    # PointCloud2 XYZRGB desde depth + RGB
    depth_to_points = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('rgb/image_rect_color', '/camera/rgb/image_raw'),
                    ('rgb/camera_info', '/camera/rgb/camera_info'),
                    ('depth_registered/image_rect', '/camera/depth/image_raw'),
                    ('points', '/camera/depth/points'),
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        set_env_vars_resources,   # <-- primero, antes de arrancar Gazebo
        gzserver_cmd,
        gzclient_cmd,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
        bridge,
        depth_camera_info,
        depth_to_points,
    ])