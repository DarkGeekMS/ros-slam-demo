# ROS SLAM Demo

A simple demonstration of __Simultaneous Localization and Mapping__ _(SLAM)_ using _ROS_ and _Gazebo_ simulator.

## Description

-   Algorithm : __Grid-Based FastSLAM__

-   Robot : __TurtleBot__

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
