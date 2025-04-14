# Trajectory Tracking Controller for Autonomous Robots
This pakcage implements a PID-based trajectory tracking controller for PX4-based autonomous robots.

Modify based on  [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [mavros](http://wiki.ros.org/mavros) ROS package. Use the following commands for installtion:
```
# install dependency
cd ~/catkin_ws/src
git clone https://github.com/HNU-CAT/pid_ctrl.git

cd ~/catkin_ws
catkin_make
```

## II. Run Controller DEMO
For tracking a circular trajectory, please run the following commands. Note that the tracking circle function is a part of [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight) package and please install it before running. Also, for simulator, you can either use the original PX4 simulator or use our PX4-based [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) package:
```
# start location
./start.sh



# launch the  tracking controller
roslaunch tracking_control tracking_control

# Manual takeoff
./start.sh

# pub circle postionb cmd
/usr/bin/python3 /tracking_controller/circle_test.py
```
The example results of running the above command can be visulized as below:

https://github.com/Zhefan-Xu/tracking_controller/assets/55560905/5a83ede2-a8a2-4c7a-a5f4-3dd3304e9ad0

https://github.com/user-attachments/assets/28321dd7-60d3-4f61-940d-227ff98c5790
## III. Controller Structure
The implemented controller follows the structure shown below:

<img width="1410" alt="controller_diagram" src="https://github.com/Zhefan-Xu/tracking_controller/assets/55560905/b3a68c70-fb1a-4ec2-be29-3cb769670490">

The related paper can be found on:

**Dario Brescianini, Markus Hehn, Raffaello D'Andrea, Nonlinear Quadrocopter Attitude Control, ETH Zurich, 2013.** [\[paper\]](https://nrotella.github.io/assets/pdf/px4_attitude_control.pdf)  


## IV. Parameters
The controller parameters can be edited and modified in ```tracking_controller/cfg/controller_param.yaml```. 

## V. ROS Topics
Here lists some important ROS topics related to the controller:

And we can change in ```controller_param.yaml```
  - Subscribing Topics:
    - ```odom_topic```: The robot odometry.
    - ```imu_topic```: The robot IMU data.
    - ```target_topic```: The desired target states.

  - Publishing Topics:
    - ```/mavros/setpoint_raw/attitude```: The command to the PX4 flight controller.
    - ```/mavros/setpoint_raw/local```: The command to the PX4 flight controller.
    - ```/tracking_controller/robot_pose```: The robot current pose.
    - ```/tracking_controller/trajectory_history```: The robot historic trajectory.
    - ```/tracking_controller/target_pose```: The target current pose.
    - ```/tracking_controller/target_trajectory_history```: The target historic trajectory.
    - ```/tracking_controller/vel_and_acc_info```: The robot state information.
    
