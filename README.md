# Jaco2 ROS Package

## Maintainer 

- [Adrian Zwiener](http://www.cogsys.cs.uni-tuebingen.de/mitarb/zwiener/welcome_e.html) <<adrian.zwiener@uni-tuebingen.de>>, [Cognitive Science](http://ai.uni-bremen.de/), University of Tübingen

## Table of contents
- [Description](#description)
- [Dependencies](#dependencies)
- [Install](#install)

## Description

This package contains a driver and ROS interfaces for the Kinova Jaco 2 manipulator. Besides, a kinematic, a dynamic model, and a moveit configuration is included. Moreover, we supply tools for calibration of the dynamic model and the Jaco's accelerometers.

It contains:
- [the diver](jaco2_driver) containg several controller and basic input and output
- [a library](jaco2_kin_dyn_lib) for kinematic and dynamic models
- [calibration tools](jaco2_calibration) to calibrate the dynamic model and the accelerometers
- [a moveit config](jaco2_moveit_config) to use the Jaco 2 with moveit
-[simple moveit apps](jaco2_simple_moveit_apps) such as random sampling
-[the model](jaco2_description) of the Jaco 2


## Dependencies
-  MoveIt!
- tracIK
- imuTK for accelerometer calibration
- Ceres  for accelerometer calibration

## Install
