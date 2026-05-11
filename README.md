ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py

ros2 run vfh_navigation waypoint_recorder

ros2 run vfh_navigation vfh_node

gz topic --echo --topic /scan

