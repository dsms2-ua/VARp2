# Image a utilizar
FROM osrf/ros:jazzy-desktop-full

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Configurar el workspace
WORKDIR /home/ros2_ws
COPY ./src /home/ros2_ws/src  

# 1. apt-get update: Descarga el catálogo de Ubuntu
# 2. rosdep update: Descarga el catálogo de librerías de ROS
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-depth-image-proc \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-rclcpp-components \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-turtlebot3-description \
      ros-jazzy-turtlebot3-gazebo \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/* && \
    rosdep init || true && \
    rosdep update

# Instalar dependencias y compilar 
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install && \
    colcon build && \
    rm -rf /var/lib/apt/lists/*

# Configuración del entorno (Bashrc)
# Agregar el source al bashrc para que ROS estén disponible al abrir la 
# terminal; el source del workspace para que los paquetes estén disponibles, y # la variable de entorno para el modelo del TurtleBot3.
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
  
# --- SECCIÓN DE COLORES Y UX ---
ENV TERM=xterm-256color
RUN echo 'export PS1="\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]# "' >> ~/.bashrc && \
    echo "alias ls='ls --color=auto'" >> ~/.bashrc && \
    echo "alias grep='grep --color=auto'" >> ~/.bashrc

# Ejecutar por defecto una terminal
CMD ["bash"]
