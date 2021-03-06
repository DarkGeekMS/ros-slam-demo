# ROS SLAM Demo

A simple demonstration of __Simultaneous Localization and Mapping__ _(SLAM)_ using _ROS_ and _Gazebo_ simulator.

## Description

-   Algorithm : __Grid-Based FastSLAM__ (from [slam-gmapping](https://github.com/ros-perception/slam_gmapping))

-   Robot : __TurtleBot3 Burger__ (from [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3))

-   Sensor : __360 Laser Distance Sensor LDS-01__

-   Environment : __Custom__

## Prerequisites

-   ROS Noetic

-   Rviz 1.14.7

-   Gazebo 11.5

-   ROS Noetic TurtleBot3 : `sudo apt install ros-noetic-turtlebot3-*`

## Usage

-   Setup `catkin` workspace :
    ```bash
    mkdir -p /home/<username>/workspace/catkin_ws/src
    cd catkin_ws/src/
    catkin_init_workspace
    cd ..
    catkin_make
    ```

-   Clone the repo packages :
    ```bash
    cd /home/<username>/workspace/catkin_ws/src
    git clone https://github.com/DarkGeekMS/ros-slam-demo.git
    ```

-   Build and source the packages :
    ```bash
    cd /home/<username>/workspace/catkin_ws
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
    source devel/setup.bash
    ```

-   Source __TurtleBot3__ model to `~/.bashrc` :
    ```bash
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
    source ~/.bashrc
    ```

-   Run the required packages for the demo :
    ```bash
    # Terminal 1 (ROS Master Node)
    roscore

    # Terminal 2 (TurtleBot3 Gazebo Simulation)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch

    # Terminal 3 (TurtleBot3 GMapping SLAM)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

    # Terminal 3 (TurtleBot3 Teleoperation)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```

-   Alternatively, you can run the demo using a single launch file _[SLOWER]_ :
    ```bash
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_slam_demo.launch
    ```

-   Save the output map :
    ```bash
    rosrun map_server map_saver -f OutMap
    ```

## Used Packages

-   [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) : for __TurtleBot3__ _Gazebo_ simulation.

-   [turtlebot3_slam](https://github.com/ROBOTIS-GIT/turtlebot3) : a wrapper for __slam_gmapping__ package to perform _SLAM_ on __TurtleBot3__.

-   [slam_gmapping](https://github.com/ros-perception/slam_gmapping) : for __Grid-Based FastSLAM__ using _ROS_ [openslam_gmapping](https://github.com/ros-perception/openslam_gmapping) _SLAM_ library.

-   [turtlebot3_teleop](https://github.com/ROBOTIS-GIT/turtlebot3) : for controlling a robot through external peripheral (used to navigate the robot during _SLAM_).

## Visual Results

<div align=center><img width="50%" height="50%" src="assets/world.png"/></div>

<div align="center">
Figure(1): World in Gazebo simulator.
</div><br>

<div align=center><img width="50%" height="50%" src="assets/robot.png"/></div>

<div align="center">
Figure(2): A 3D view of TurtleBot3 in Gazebo simulator.
</div><br>

<div align=center><img width="50%" height="50%" src="assets/map.png"/></div>

<div align="center">
Figure(3): A view of a portion of result occupancy grid map (OGM) in Rvis.
</div><br>
