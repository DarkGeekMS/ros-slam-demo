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

-   Run the required packages for the demo.

## Resources

-   [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) : contains the required packages for __TurtleBot3__ robot model and its controls.

-   [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) : contains the required packages for __TurtleBot3__ robot simulation using _Gazebo_.

-   [slam_gmapping](https://github.com/ros-perception/slam_gmapping) : contains the required packages for __Grid-Based FastSLAM__ using _ROS_ [openslam_gmapping](https://github.com/ros-perception/openslam_gmapping) _SLAM_ library.

-   [joystick_drivers](https://github.com/ros-drivers/joystick_drivers) : contains the required packages for controlling a robot through external peripheral (used to navigate the robot during _SLAM_).

-   [Grid-based-FastSLAM](https://github.com/AndresGarciaEscalante/Grid-based-FastSLAM) : a simple demo example for __Grid-Based FastSLAM__ using an older _ROS_ version (can be used for connecting the packages).
