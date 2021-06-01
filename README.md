# ROS SLAM Demo

A simple demonstration of __Simultaneous Localization and Mapping__ _(SLAM)_ using _ROS_ and _Gazebo_ simulator.

## Description

-   Algorithm : __Grid-Based FastSLAM__ (from [slam-gmapping](https://github.com/ros-perception/slam_gmapping))

-   Robot : __TurtleBot3__ (from [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3))

-   Sensor : __Laser Range Sensor__

-   Environment : __Custom__

## Prerequisites

-   ROS

-   Gazebo

-   Rviz

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

-   Run the required packages for the demo :
    ```bash
    # Terminal 1 (ROS Master Node)
    roscore

    # Terminal 2 (TurtleBot3 Gazebo Simulation)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_house.launch

    # Terminal 3 (TurtleBot3 GMapping SLAM)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

    # Terminal 3 (TurtleBot3 Teleoperation)
    cd /home/<username>/workspace/catkin_ws
    source devel/setup.bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
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
