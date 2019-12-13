# PoE Final Project - Telemetric Robotic Hand
## Quickstart

Typing these commands into your terminal will make it so that you have the repo and all the code you need on your computer.
```
git clone https://github.com/EverardoG/poe_final_project.git
git submodule init
git submodule update
```

To make your dependecies work, all you have to do is copy all of the folders inside of `dependencies` into wherever your `Arduino/libraries` folder is.

## Context
This repo contains all of the software for a Principles of Engineering final course project. This project was approximately 8 weeks long, and focused on systems integration, project management, and project iterations. [Read more about the project and the overall robot system here.](poe.olin.edu/2019/rgpt) To read more about the software specifically, read on!

## Software Overview
The primary scripts used for our final demo are in `poe_final_project/robot_code/system_demo/`, with our primary arduino script being `system_demo.ino`. This script is broken down into `SENSE`, `THINK`, and `ACT` sections. Each section contains the relavent software for that section. The `SENSE` section collects data from all the sensors attached to the system - this includes the IMU and bend sensors attached to the glove, and the extra not-implemented IMU attached to the system. The `THINK` section contains all of the software for processing these sensor inputs and turning them into meaningful inputs for the actuators of the system. The `ACT` section of the software updates the stepper motors and servo motors attached to the system. 

For organization's sake, the software is also broken down into a `RobotHand` class stored in `RobotHand.cpp` and `RobotHand.h`, and a `HumanHand` class stored in `HumanHand.cpp` and `HumanHand.h`. Both of these classes are included in the `rgpt_lib.h` file to streamline our main script, `system_demo.ino`. The `HumanHand` class is responsible for all of the sensing functionality apparent in the glove. `HumanHand` provides a convenient interface for pulling sensor input into the Arduino as meaningful data. The `RobotHand` class is responsible for all of the actuation functionality of the system. `RobotHand` provides a convenient interface to commanding the servo and stepper motors on the system in real-time. 

## Dependencies
This software relies on various arduino libraries that made it possible for us to build on the shoulders of giants with this project. These libraries are listed below along with a link to the corresponding github repo, and a description of what we used the lirbary for.

* **Adafruit_9DOF** - [Working with 9DOF data from Adafruit IMUs](https://github.com/adafruit/Adafruit_9DOF/tree/9ec1baf318969267c3ce564747960036f855ec6d)
* **Adafruit_AHRS** - [Attitude and Heading for IMUs](https://github.com/adafruit/Adafruit_AHRS/tree/a01de978b6000ab114b1991f57d5fad7dc4c8555)
* **Adafruit_FXAS21002C** - [Interfacing with Adafruit Adafruit_FXAS21002C Gyroscope](https://github.com/EverardoG/Adafruit_FXAS21002C/tree/20818ce1deeaef950cbfc11850b595f8139e3850)
* **Adafruit_FXOS8700** - [Interfacing with Adafruit FSOX8700 Accelerometer/Magnetometer](https://github.com/EverardoG/Adafruit_FXOS8700/tree/52b91f678ff82f834a390652682b0683d01dba6f)
* **Adafruit_L3GD20_U** - [Interfacing with Adafruit Adafruit_L3GD20_U Gyroscope](https://github.com/adafruit/Adafruit_L3GD20_U/tree/eef24c1991f1ccb318dff082160172fa4591f9a8)
* **Adafruit_LSM303DLHC** -  [Interfacing with Adafruit Adafruit_LSM303DLHC Accelerometer/Magnetometer](https://github.com/adafruit/Adafruit_LSM303DLHC/tree/38febfac791fb9d75a615574fb4f5c5ee832372b)
* **Adafruit_Sensor** - [Unified sensor library for Adafruit sensors](https://github.com/adafruit/Adafruit_Sensor/tree/6f4785cd498f07144a3f5c4e452aaa238c0a7c36)
* **Filters** - [Filtering sensor data](https://github.com/JonHub/Filters/tree/aeb294b4a66cf5dc30d5998cc221115dec95eb80)
* **FlexyStepper** - [Dynamic Stepper Control](https://github.com/Stan-Reifel/FlexyStepper)
