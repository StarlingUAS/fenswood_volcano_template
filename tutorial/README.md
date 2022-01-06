# Fenswood Volcano Template Tutorials

## Introduction

Inspired by the [minimal ROS2 examples](https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber), this tutorial takes you through several iterations of code with almost the same functionality.  A drone takes off, moves to a target location, and then flies home.  In the later `perception` example, some very basic information is gathered from its camera image.  Together, these examples give you all the building blocks you will need to carry out the project.

> You can also add your own functionality.  As well as the blocks of a minimum working solution, the examples show you how to add ROS packages and Python libraries.  You can use this facility to exploit some of the many third-party packages and libraries available in ROS and Python.

The template is built on the [Starling](https://github.com/StarlingUAS) framework which combines open-source components to provide a realistic, cross-platform simulation for drone flight:
 - [Gazebo](http://gazebosim.org/), a physical and visual simulation package
 - [Ardupilot](https://ardupilot.org/), a free autopilot software with [MAVLINK](https://mavlink.io/en/) interfacing and [simulation](https://ardupilot.org/copter/docs/common-simulation.html) capability
 - [ROS](https://www.ros.org/), a popular software framework for robot interfacing, plus the [MAVROS](http://wiki.ros.org/mavros) package for connecting MAVLINK drones to ROS. 
 - [OpenCV](https://opencv.org/), an image processing library with Python support
 - [Docker](https://www.docker.com/), a system for packaging and deploying 

> Surely there's a simpler way?  Almost certainly, but you wouldn't learn so much.  These are the standard tools of the robotics trade, and while it'll take you a while to learn them, the skills you get will go far beyond this project.
 


To simplify changing, each iteration lives in a different `git` branch

