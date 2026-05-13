ros2 run vfh_navigation waypoint_recorder

# Para lanzarlo todo a la vez
ros2 launch vfh_navigation navigation.launch.py


ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py
ros2 run vfh_navigation vfh_node
ros2 run vfh_navigation metrics_node 

ros2 topic echo /lap/metrics



