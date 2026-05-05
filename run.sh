xhost +local:
docker compose up -d --build
docker exec -it var_container bash -lc "source /opt/ros/jazzy/setup.bash && cd /home/ros2_ws && colcon build --symlink-install && source install/setup.bash && exec bash"
