# ros_music_adapter

This project provides adapters to couple [ROS](http://www.ros.org)
with [MUSIC](https://github.com/INCF/MUSIC).
While ROS is a powerful communication framework for robotic simulation,
MUSIC allows to coordinate and communicate between different
time-synchronous simulators and natively supports spiking communication
channels.

For instance, this set of adapters allow to couple
[Gazebo](http://gazebosim.org), a physics simulation software,
with simulators for spiking neural networks, like 
[NEST](http://nest-simulator.org/) and [NEURON](http://neuron.yale.edu/neuron/).

## Installation

The adapters are provided as a catkin packages, which means the
installation is as easy as dropping the repository itself in
the catkin workspace.

### Dependencies

- MUSIC (https://github.com/INCF/MUSIC)
- OpenMPI
- GSL BLAS
- Json-cpp
- pthread

