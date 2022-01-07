# Fenswood Volcano Template Tutorials

## Introduction

You will learn how to write code in Python to interface with the (simulated) drone using ROS.  In particular, you will learn the newer ROS2 methods, which offer improved security and scalability.  On the way, you will learn the fundamentals of controlling a drone through a standard autopilot.

> You will also need to do little bits of Linux and Docker 'magic' to make the application work.  You are not expected to master these aspects: ask for help if they cause problems.  The primary learning objectives here are drone control, Python and ROS.  If you are interested in more details, see the [Starling overview](starling.md).

In the examples, a drone takes off, moves to a target location, and then flies home.  In the later `perception` example, some very basic information is gathered from its camera image.  Together, these examples give you all the building blocks you will need to carry out the project. 

## How to use this tutorial

Inspired by the [minimal ROS2 examples](https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber), this tutorial takes you through several iterations of code with almost the same functionality.  A full working example is provided in each case with a detailed description of the code.  The section below shows the contents and learning obectives for each stage of the tutorial.  You are advised to simply work through from start to finish _but_ you might find some stages easier than others, depending on your past experience.

Each tutorial stage comes with some exercises for you to add to the code yourself.  To do the exercises, first use `git checkout <branch>` to activate the relevant branch of the repository, then edit the appropriate files.  To test your solutions, the simulation is always started using `docker-compose up --build` and then stopped using `Ctrl+C`.  Use the terminal window output, foxglove studio, or the Docker Desktop (Windows and Mac only) to inspect the results.

> Don't forget the `--build`.  Otherwise, your changes don't get built into the simulation, and you're just running the same code over and over.

## Contents

1. [Drone Control](drone_control.md)
    - Learn the key steps in flying a drone using Ardupilot and ROS
 




