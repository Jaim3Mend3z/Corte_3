# Usar una imagen base oficial de ROS 2 Jazzy (más genérica)
FROM ros:jazzy

# Evitar preguntas interactivas durante la instalación de paquetes
ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias del sistema
RUN apt-get update && apt-get install -y \
    python3-pip \
    x11-apps \
    libgl1 \
    libegl1 \
    mesa-utils \
    python3-pynput \
    && rm -rf /var/lib/apt/lists/*

# Instalar PyBullet usando pip3
RUN pip3 install pybullet --break-system-packages

# Crear y establecer el directorio de trabajo para el workspace de ROS 2
WORKDIR /ros2_ws

# --- CAMBIO IMPORTANTE AQUÍ ---
# 1. Crear explícitamente el directorio src dentro del workspace
RUN mkdir src

# 2. Copiar tu paquete ROS 2 al directorio 'src' que acabamos de crear
#    La ruta de destino es relativa al WORKDIR (/ros2_ws), por lo que será /ros2_ws/src/mi_paquete_quadrupedo_ros2
COPY ./mi_paquete_quadrupedo_ros2 ./src/mi_paquete_quadrupedo_ros2 
# --- FIN DEL CAMBIO IMPORTANTE ---

# Construir el workspace de ROS 2
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Comando por defecto
CMD ["bash"]
