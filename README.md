# FSAE Ackermann Steering Kinematics

This repository contains MATLAB scripts used to simulate and analyze the steering geometry for a Formula SAE vehicle. It calculates the Ackermann steering angles for both the left and right wheels based on the steering rack stroke.

## Features
* Calculates real-time steering angles using numeric root finding (`fzero`).
* Plots Ackermann geometry (Steering Angle vs. Rack Stroke).
* Validates maximum stroke angles against mechanical design diagrams.

## Parameters
The simulation uses the following default geometric parameters:
* **Kingpin Transversal Distance (W_kp):** 1101.21 mm
* **Steering Arm Length (R_arm):** 71.00 mm
* **Rack Longitudinal Offset (Y_rack):** 40.00 mm
* **Max Rack Stroke:** 31.75 mm
* **Static Kingpin Angle:** 15.78°

## Getting Started
1. Clone this repository.
2. Open MATLAB and navigate to the `src/` directory.
3. Run the `Steer_Stroke_angle_realvalues_twowheels.m` script.
4. The script will automatically generate the kinematic plots and output the max stroke validation to the command window.

## Dependencies
* MATLAB (Requires the Optimization Toolbox for `fzero` function).
