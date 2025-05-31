FROM ros:noetic

# Instalar herramientas necesarias
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-rqt \
    ros-noetic-rqt-graph \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copiar código
COPY ./scripts /catkin_ws/src/ros_docker_example/scripts
COPY ./launch /catkin_ws/src/ros_docker_example/launch

# Crear archivo CMake mínimo
RUN echo "cmake_minimum_required(VERSION 3.0.2)\n\
project(ros_docker_example)\n\
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)\n\
catkin_package()\n\
catkin_install_python(PROGRAMS scripts/talker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})" \
> /catkin_ws/src/ros_docker_example/CMakeLists.txt

# Compilar
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make"

# Activar entorno
CMD ["/bin/bash", "-c", "source devel/setup.bash && roslaunch ros_docker_example basic.launch"]
