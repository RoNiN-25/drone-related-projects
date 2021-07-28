# drone-related-projects
This repository contains some of the drone relates projects I've worked on.
There are three projects here:
* __plutox__ contains a simulation of the the PlutoX drone with a PID controller and a custom plugin
* __PX4__ contains a simple _IRIS Quadrotor_ simulation using _PX4 SITL_ and the _Gazebo Simulator_
* __swarm__ contains a simulation of a _Drone swarm for tracking a target_ using _Hector Quadrotor_

Most dependencies are along with the package. But, the __PX4__ projects need the __PX4-Autopilot__ repository cloned, built and sources
The link to the PX4-Autopilot repository can be found here:
https://github.com/PX4/PX4-Autopilot

##Instructions to use
Each folder contains the _src_ folder of a ROS package with associated _CMakeLists.txt_ for building them. Clone the repository and put each _src_ folder in a different workspace. Use `catkin build` command for PX4 and `catkin_make` command for plutox and swarm to build and compile the scripts.

To run:
* __PX4__
  * In one terminal run:
	`roslaunch drone_trajectory_follow traj_follow.launch`
  * In another terminal run:
	`rosrun drone_trajectory_follow TrajFollow.py`

* __swarm__
  * In one terminal run:
	`roslaunch swarm_search swarm_test.launch`
  * In another terminal run:
	`roslaunch swarm_seach launch_controllers.launch`

* __plutox__ (WIP)
  * In one terminal run:
	`roslaunch fly_bot_cpp pluto_gazebo.launch`
  * In another terminal run:
	`rosrun fly_bot_cpp TrajFollow.py`
	
