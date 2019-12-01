Human Robot Interaction via virtual reality and ROS integration
=============================

#### *Victor Ozoh -- MS in Robotics Final Project - Northwestern University*


## Overview
The final goal of this project is to develop a sword fighting robot. In addition, secondary goals include
exploring human robot interaction using tools and techniques from Computer Vision, Virtual Reality and Neural Networks.
If time permits, secondary goals include:
- Human-Robot interaction via virtual reality. For example Tele-operation.

The goal for the end of this quarter is to learn motion planning in ROS using the Moveit package.

## Milestones
- Develop sword-sensor interface to Sawyer robot (milestone 1)
- ROS node to accurately determine motion of sword (milestone 2)
- Sensors on human arm and ROS node to determine motion of human arm (milestone 3)
- Motion planning ROS node to control Sawyer arm to parry human sword (milestone 4)
- Deep Learning/ Reinforcement Learning ROS node to learn and anticipate human motion (milestone 5)

## Components and Tools
- Ubuntu 18.04
- ROS Melodic
- SteamVR
- Sawyer Robot
- Modern Robotics Python Library

## Installation Instructions
- Follow the [instructions here](https://github.com/robosavvy/vive_ros) to install the vive_ros package and to install steam on your machine.
- Disable headset requirement by editing the file at the location listed below
```
~/.steam/steam/steamapps/common/SteamVR/resources/settings/default.vrsettings
```
Change the saying ```"requireHmd":true ``` to ```"requireHmd": false ```
## Package References
- [ROS Package for publishing HTC Vive device locations](https://github.com/robosavvy/vive_ros)


## Timeline
There are 10 weeks in the quarter starting September 24, 2019. Tentative schedule is to spend 2 weeks on each
milestone listed above.
- 09/24/2019 Milestone 1
- 10/08/2019 Milestone 2
- 10/22/2019 Milestone 3
- 11/05/2019 Milestone 4
- 11/19/2019 Milestone 5
- 12/03/2019 Explore Virtual Reality control of Sawyer robot
