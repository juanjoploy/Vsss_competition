# Usa la imagen oficial de ROS 2 Humble con Gazebo
FROM osrf/ros:humble-desktop

# Instalar dependencias adicionales
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Configurar el entorno
ENV ROS_DISTRO humble
ENV ROS_DOMAIN_ID 0
SHELL ["/bin/bash", "-c"]

# Crear un directorio de trabajo
WORKDIR /ros2_ws

# Copiar el contenido del workspace (opcional, si no usas bind mounts)
COPY . /ros2_ws/

RUN bash -c "source /opt/ros/humble/setup.bash && \
    export CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH && \
    rm -rf build install log && \
    colcon build"


# Fuente de setup en el entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]
