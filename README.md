# Group Design Project -> Concept and Prototype of a Self-Landing Rocket
# Project Overview
The aim of this project is to design, develop and test a model rocket capable of autonomously controlling its descent, the project involves a combination of propulsion system integration, flight dynamics modelling, and guidance, navigation and control (GNC) systems to enable autonomous flight and descent. Control of the rocket was achieved via thrust vector control (TVC), where the flight software controls two servo-motor actuators in the pitch and yaw directions offering a restoring torque to the rocket during powered flight. A parachute mechanism was developed and released at apogee to slow down the rocket on descent and keep stability with a landing legs mechanism deploying further close to the gound. Finally, a second motor re-activates slowing the rocket close to rest.

# This repository contains all scipts, simulations and flight software code regarding the flight dynamics modelling, and electronics and control side of the project.

The mission plan for the rocket is:
1. Powered flight to a 70m apogee
2. Deployment of parachute at apogee
3. Deployment of landing legs on descent
4. Re-activation of a second motor bringing the rocket to rest

# Electronic Systems Overview
The rocket features a combination of IMUs and sensors, measuring anlges and angular acceleration, velocities, accelerations and altitude, providing crucial inputs for the control systems and informing the rocket of its vertical position for release of the parachute, landing legs and second motor. 

# Flight Capabilities
The flight software is programmed onto an Arduino microcontroller performing the following tasks:

# State Detection
This concerns the detection of various states:
1. PREFLIGHT
2. LAUNCH
3. BURNOUT
4. APOGEE
5. DESCENT
6. LANDING
These are crucial to ensure adherence to the mission plan and timing of mechanism deployments. Practically this is done through a state machine architecture where the rocket begins in PREFLIGHT and can only transition state sequentially based on programmed state detection logic. At apogee the flight software releases two servo-motors in the nose cone allowing a parachute to release and unfurl, on descent the flight software heats up a nichrome wire burning a release mechanism for the landing legs.

# Control Systems
The flight software features a combinations of PD controllers and Kalman filters for state estimation and control of actuators. The controller takes in angle inputs and angular rate from the IMU, vectoring the TVC mechanism to account for any deviation from vertical.

# Data Acquisition and Live Updates
Data such as position, velocity, acceleration, state, and controller response are recorded via an on-board SD card. The Arduino microcontroller can interface with a laptop through a TCP connection allowing for live state updates, emergency commands and initial ignition of the rocket motor from a remote location. This capability allows for rapid sub-system testing, iteration and prototyping as well as functional use during launch.

# Repository Overview
