# :omnibot: This is a project of ROBOTIS TurtleBot3 virtual model with Mecanum wheels in Gazebo. :omnibot:

## Overview

Package provides operable model of TurtleBot3 by ROBOTIS with Mecanum wheels in Gazebo.

## Features

- Mecaum wheels with 8 rollers are desined:

 <img src="https://github.com/RoboHoboMobo/OmniTurtle/blob/master/omniturtle_description/pics/mecanum_bare_rviz.png" width="40%"/>  
 
- TurtleBot3 Burger model with 4 Mecanum wheels is desined:

 <img src="https://github.com/RoboHoboMobo/OmniTurtle/blob/master/omniturtle_description/pics/burger.png" width="40%"/>  
 
- Simple keyboard teleoperation console node is implemented:

 <img src="https://github.com/RoboHoboMobo/OmniTurtle/blob/master/omniturtle_description/pics/keyboard_node.png" width="40%"/>  

<!-- ## References

Check the [paper](https://gitlab.com/LIRS_Projects/Simulation-fiducial-markers-experimets/tree/master/paper) about our virtual experiments -->

## Prereqirements

This repo is designed and tested on **Ubuntu 16.04** with **ROS Kinetic**.

Following resources were used:

- **Original model of ROBOTIS TurtleBot3** : *turtlebot3_simulations package* [ [Git](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) ]
- **NEXUS ROBOT Mecanum wheel as prototype** : *link to online-store* [ [URL](https://www.nexusrobot.com/product/a-set-of-60mm-lego-compatible-mecanum-wheels-4-piecesbearing-rollers-14144.html) ]
- **Mecanum wheel STL model** : *link to web-site* [ [URL](https://grabcad.com/library/60mm-mecanum-wheel-1) ]

Also, controller packages must be installed used following commands:

```bash
sudo apt install ros-kinetic-joint-state-controller
sudo apt install ros-kinetic-position-controllers
sudo apt install ros-kinetic-gazebo-ros-control
sudo apt install ros-kinetic-controller-manager
sudo apt install ros-kinetic-effort-controllers
```

## Usage

Run nodes in this order:
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosrun omniturtle_control omniturtle_manager
rosrun omniturtle_control omniturtle_keyboard_node
```
