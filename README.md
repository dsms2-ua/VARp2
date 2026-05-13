ros2 run vfh_navigation waypoint_recorder

# Para lanzarlo todo a la vez
ros2 launch vfh_navigation navigation.launch.py


ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py
ros2 run vfh_navigation vfh_node
ros2 run vfh_navigation metrics_node 

ros2 topic echo /lap/metrics

# Instalar yolo (hazlo solo una vez y hazte commit de la imagen)
pip3 install ultralytics "numpy<2" --ignore-installed numpy --break-system-packages

# Para hacer deteccin
ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py
ros2 run detector_pkg detector_node
ros2 run rqt_image_view rqt_image_view
