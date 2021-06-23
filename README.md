rg2_ft_ros
============

This repository contains ROS packages needed for using the OnRobot RG2-F/T gripper.


Physical Installation
-----------------------

The gripper must be connected to the OnRobot Compute Box, which in turn must be connected to the computer
using a normal ethernet cable.  The gripper must be powered following the manufacturer's instructions.


rg2_ft_description
--------------------

This package contains the URDF and supporting files for representing the gripper on a robot.


rg2_ft_driver
--------------

This package contains the underlying ROS driver for the gripper.  The driver requires PyModbus.
