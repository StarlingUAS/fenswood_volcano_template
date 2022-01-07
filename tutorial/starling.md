# Brief outline of the Starling framework

The template is built on the [Starling](https://github.com/StarlingUAS) framework which combines open-source components to provide a realistic, cross-platform simulation for drone flight:
 - [Gazebo](http://gazebosim.org/), a physical and visual simulation package
 - [Ardupilot](https://ardupilot.org/), a free autopilot software with [MAVLINK](https://mavlink.io/en/) interfacing and [simulation](https://ardupilot.org/copter/docs/common-simulation.html) capability
 - [ROS](https://www.ros.org/), a popular software framework for robot interfacing, plus the [MAVROS](http://wiki.ros.org/mavros) package for connecting MAVLINK drones to ROS. 
 - [OpenCV](https://opencv.org/), an image processing library with Python support
 - [Docker](https://www.docker.com/), a system for packaging and deploying 

> Surely there's a simpler way?  Almost certainly, but you wouldn't learn so much.  These are the standard tools of the robotics trade, and while it'll take you a while to learn them, the skills you get will go far beyond this project.  This way also means we can do more with Starling, like migrate controllers from simulation to actual flight.