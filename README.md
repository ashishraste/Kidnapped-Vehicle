# Kidnapped Vehicle

Localization of a kidnapped vehicle using Particle filter.

<img src="particle-filter.gif?raw=true">

# Overview
This project is part of [Udacity's Self-Driving Car Nanodegree program](https://www.udacity.com/drive).
A 2-dimensional [Particle filter](https://en.wikipedia.org/wiki/Particle_filter)
has been implemented in this project, where sensor and control measurements of a
simulated car are used to predict its location. An initial GPS location is
provided by the simulator, after which the vehicle relies on the filter to
localize itself.

A [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45)
provided by Udacity is used to generate and visualise measurements and motion
of a car. More information on installation and usage of the simulator with
the executable can be found in the seed-project setup by Udacity [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

## Dependencies
1. CMake >= 3.5
2. Make >= 4.1
3. Eigen 3.3.5
4. gcc/g++ >= 4.1
5. [uWebSockets](https://github.com/uNetworking/uWebSockets)

## Build and Run Instructions
1. From the parent directory, create the build
```bash
sh clean.sh
sh build.sh
```
2. Launch the simulator
3. Run the Particle filter executable
```
sh run.sh
```
