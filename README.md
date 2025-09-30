# Teach and repeat trajectory using a Turtlebot3 Waffle Pi
‭C++ coding evaluation for Senior Robotics Sofrware Engineer at Intermodalics (Leuven, Belgium).  

# Requirements
Even though there is a Dockerfile to build and run the program, it works in Ubuntu 24.04.3 LTS and ROS 2 Jazzy.


# Steps to reproduce
1. Clone the repository to the folder you like.
   
2. Build the Dockerfile with:
   ```shell 
	 sudo docker build -t intermodalics_teachrepeat .
   ```
3. While building, please allow remote connections so that docker can open displays on the host:
   ```shell 
	 xhost local:root
   ```
4. Launch the Dockerfile in case you have GPUs on your machine (otherwise, remove --gpus "all", -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} and -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all}):
   ```shell 
	 sudo docker run --shm-size=1g --ulimit memlock=-1 --ulimit stack=67108864 --gpus "all" --rm -it --name intermodalics_teachrepeat -v ~/Desktop/intermodalics/:/intermodalics -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} intermodalics_teachrepeat
   ```   
5. Open two extra terminals to handle all the callings:
   ```shell 
	 sudo docker exec -it intermodalics_teachrepeat /bin/bash
   ```
6. In all three terminals, access the workspace, and source it:
   ```shell 
	 cd /root/turtlebot3_ws/ && colcon build && source install/setup.bash
   ```
7. Terminal 1 will launch Gazebo with waffle_pi:
   ```shell 
	 ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
8. Terminal 2 will launch the teleoperation node:
   ```shell 
	 ros2 run turtlebot3_teleop teleop_keyboard
   ```	
9. Terminal 3 will launch the get_trajectories node (once the robot is positioned on the desired start position) and will save the file at /root/turtlebot3_ws/path.csv. Once the trajectory is recorded, kill all 3 processes in the three terminals:
   ```shell 
	 ros2 run intermodalics_teachrepeat get_trajectories --ros-args -p parent_frame:=odom -p child_frame:=base_footprint -p output_csv:=path.csv
   ```	
10. To visualize the trajectory, launch rviz2 and load the topic /robot_trajectory to visualize it in a terminal after executing the following command. Other option would be open the .csv file and edit the trajectory:
   ```shell 
	 ros2 run intermodalics_teachrepeat visualize_trajectories
   ```
11. To replay the trajectory, relaunch Gazebo with waffle_pi (terminal 1) and the following command in terminal 2:
   ```shell 
	 ros2 run intermodalics_teachrepeat replay_trajectories path.csv
   ```
