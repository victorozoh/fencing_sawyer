Human Robot Interaction via virtual reality and ROS integration
=============================

#### *Victor Ozoh -- MS in Robotics Fall Project - Northwestern University*


## Overview
The final goal of this project is to develop a fencing robot using the Sawyer robot from Rethink Robotics. The system makes use of the HTC Vive lighthouse and tracker to localize the arm movements of the opponent.

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
Change the line saying ```"requireHmd":true ``` to ```"requireHmd": false ```
- Edit the line saying ```activateMultipleDrivers:false``` to ```activateMultipleDrivers:true```
- Set ```forcedDriver:null``` and ```forcedHmd:""```
- Open the following file using ``` gedit ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings``` and set the ```enable:true```
- Ensure you have full access permissions to ```/dev/hidraw*``` which is where the Vive wireless adapter will be visible
- You may need to install for your version of linux

## Vive node
This node is from the package ```vive_ros``` and is responsible for publishing the transforms of the vive tracker with respect to the lighthouse. It also publishes a twist message on the ```/vive/twist1``` topic. I edited the ```vive.launch``` file in the ```vive_ros```package to provide a fixed offset of the lighthouse with respect to the world frame in ROS.

## Fencer node
This node implements the fencing logic in a class named ```Fencer```.
It subscribes to the ```/vive/twist1``` topic in order to detect if the opponent is moving or not.
The logic has the robot a defensive mode. Hence the robot mostly attempts to parry the opponent's attack once opponent is within a given distance from the robot.


## Package References
- [ROS Package for publishing HTC Vive device locations](https://github.com/robosavvy/vive_ros)


## How to run
There is a single launch file. From the workspace containing the package, run:

```roslaunch fencing_sawyer fencing.launch```
