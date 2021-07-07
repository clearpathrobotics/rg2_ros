rg2_ft_ros
============

This repository contains ROS packages needed for using the OnRobot RG2-F/T gripper.

![RG2-FT URDF](doc/rg2_ft_urdf.png "The RG2-FT URDF")

Physical Installation
-----------------------

The gripper must be connected to the OnRobot Compute Box, which in turn must be connected to the computer
using a normal ethernet cable.  The gripper must be powered following the manufacturer's instructions.


rg2_description
--------------------

This package contains the URDF and supporting files for representing the gripper on a robot.

This package contains meshes for the RG2 gripper authored by Sharath Jotawar as part of the now-archived
ur10_rg2_ros package, available here: https://github.com/sharathrjtr/ur10_rg2_ros

The main body of the hand extends along the X-axis.  The Left and Right fingers match the labels on the physical
gripper, and the coordinate frames of the fingertips are rotated to match these same labels.

For simplicity the hand's joints are not fully modelled; on the real gripper the first finger links are compound
joints that have expanding segments physically keep the fingertips in alignment.  This package treats all 4 finger
joints as independent revolute joints.  The driver is responsible for pubishing the underlying joint angles.

Note that the physical hand supports setting independent default angles for each fingertip.  At present this is not
supported; the URDF and driver both assume that the fingertip angles are set to 0 and that the fingers always move
in a synchronized fashion.


rg2_driver
--------------

This package contains the underlying ROS driver for the gripper.  The driver requires PyModbus.


rg2_msgs
--------------

This package contains additional ROS messages used by the gripper driver.
