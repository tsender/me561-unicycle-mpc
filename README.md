me561-unicycle-mpc
==================

This is a ROS repo intended to develop a MPC controller for the turtlebot.

# Building / Installation

## Dependenies

This library requires ROS and [CVXOPT](https://cvxopt.org/install/index.html)

## Building
The recommended way to build/install this library is with [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

Clone this repo into a src folder and build with catkin tools:

    cd ~/catkin_ws/
    git clone --recursive https://github.com/tsender/me561-unicycle-mpc.git src
    catkin init
    catkin config --install
    catkin build
    
