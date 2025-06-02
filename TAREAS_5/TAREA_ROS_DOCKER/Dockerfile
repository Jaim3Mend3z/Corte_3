# Usar una imagen base oficial de ROS 2 Jazzy (más minimalista)
FROM ros:jazzy

# Evitar preguntas interactivas durante la instalación de paquetes
ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias del sistema:
# - python3-pip: para instalar paquetes de Python
# - x11-apps: utilidades X11, bueno para probar la GUI
# - libgl1, libegl1, mesa-utils: Librerías gráficas estándar para OpenGL/GUI
# - python3-pynput: para el control por teclado
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

# Copiar tu paquete ROS 2 al directorio 'src' del workspace
COPY ./mi_paquete_pybullet_ros2 ./src/mi_paquete_pybullet_ros2

# Construir el workspace de ROS 2
# El comando '. /opt/ros/jazzy/setup.sh' carga el entorno de ROS antes de compilar
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Configurar el entorno para que se cargue al iniciar un contenedor
# Y establecer el comando por defecto a bash para poder ejecutar comandos manualmente
CMD ["bash"]
