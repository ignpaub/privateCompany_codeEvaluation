# Start from already available image - ROS2 & Ubuntu 24.04 LTS
FROM osrf/ros:jazzy-desktop-noble

# Update and reinstall ROS2 (have latest updates)
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install necessary libraries for GUI applications (open graphic display)
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Make sure no questions are asked while installing packages (for example, region)
ENV DEBIAN_FRONTEND=noninteractive

# Install some common packages for performing later installations
RUN apt-get update && apt-get install -y apt-utils bash-completion build-essential curl git gnupg lsb-release python3-colcon-common-extensions sudo

# Install ROS2 dependent packages (gazebo_sim, cartographer and navigation2) 
RUN apt-get update && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && apt-get update && apt-get install -y gz-harmonic ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install turtlebot3 packages
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src/ && git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN source /opt/ros/jazzy/setup.bash && cd ~/turtlebot3_ws/src/ && ros2 pkg create --build-type ament_cmake privateCompany_teachrepeat && source /opt/ros/jazzy/setup.bash && cd ~/turtlebot3_ws && colcon build --parallel-workers $(nproc) --symlink-install
COPY ./CMakeLists.txt /root/turtlebot3_ws/src/privateCompany_teachrepeat/CMakeLists.txt
COPY ./package.xml /root/turtlebot3_ws/src/privateCompany_teachrepeat/package.xml
COPY ./get_trajectories.cpp /root/turtlebot3_ws/src/privateCompany_teachrepeat/src/get_trajectories.cpp
COPY ./visualize_trajectories.cpp /root/turtlebot3_ws/src/privateCompany_teachrepeat/src/visualize_trajectories.cpp
COPY ./replay_trajectories.cpp /root/turtlebot3_ws/src/privateCompany_teachrepeat/src/replay_trajectories.cpp

RUN source /opt/ros/jazzy/setup.bash &&\
    cd ~/turtlebot3_ws &&\
    colcon build --parallel-workers $(nproc) --symlink-install

# Source ROS2 and created workspace, as well as setting ROS_DOMAIN_ID
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc && echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc && source ~/.bashrc
