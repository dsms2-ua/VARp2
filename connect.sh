docker exec -it var_container bash -lc "source /opt/ros/jazzy/setup.bash && cd /home/ros2_ws && source install/setup.bash 2>/dev/null || true; exec bash"
