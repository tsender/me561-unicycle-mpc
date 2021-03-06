me561-unicycle-mpc
==================

This is a ROS repo intended to develop a MPC controller for the turtlebot.

# Building / Installation

## Dependenies

This library requires ROS and [CVXPY](https://www.cvxpy.org/install/index.html)
Note: to install CVXPY, if you get an error about user permissions, execute 
        
        pip install cvxpy --user

## Building
The recommended way to build/install this library is with [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

Clone this repo into a src folder and build with catkin tools:

    mkdir -p ~/catkin_ws/me561-unicycle-mpc/
    cd ~/catkin_ws/me561-unicycle-mpc/
    git clone --recursive https://github.com/tsender/me561-unicycle-mpc.git src
    catkin init
    catkin build
    
## Post Setup
Add these lines to the end of your ~/.bashrc file:

    source ~/catkin_ws/me561-unicycle-mpc/devel/setup.bash
    export TURTLEBOT3_MODEL=burger
    
Don't forget to source your ~/.bashrc file for the changes to take effect.

# Running the Code
In one terminal, do

    roslaunch turtlebot_mpc turtlebot_mpc.launch
    
In another, do

    rosrun rqt_gui rqt_gui

In rqt_gui, publich a Bool message only ONCE on topic /go_turtlebot_go with a value of True, and then the robot will start moving
