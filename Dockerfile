# Image a utilizar
FROM osrf/ros:jazzy-desktop-full

# Evitar prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Configurar el workspace
WORKDIR /home/ros2_ws

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

# --- SECCIÓN DE COLORES Y UX ---
# 1. Definir que la terminal soporta colores
ENV TERM=xterm-256color

# 2. Configurar el prompt (PS1) y los alias de colores para ls/grep
RUN echo 'export PS1="\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]# "' >> ~/.bashrc && \
    echo "alias ls='ls --color=auto'" >> ~/.bashrc && \
    echo "alias grep='grep --color=auto'" >> ~/.bashrc

# Agregar el source al bashrc para que ROS esté disponible al abrir la terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Ejecutar por defecto una terminal
CMD ["bash"]
