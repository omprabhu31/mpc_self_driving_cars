# Adaptive MPC Control for Obstacle Avoidance
### Part of  Dual Degree Project - Stage I

This folder contains the relevant code files for simulation of an adaptive MPC controller for generating constraints for obstacle avoidance. The simulation is carried out using Simulink, and you will need the version of Simulink supplied with MATLAB R2023b in order to run the files.

Certain function blocks pertaining to the simulation have been adapted from a [web-based tutorial](https://in.mathworks.com/help/mpc/ug/obstacle-avoidance-using-adaptive-model-predictive-control.html) on the Mathworks Help Center, namely the sections of code used for plotting the obstacle avoidance maneuver, determination of relationships between some of the simulation archutecture used for storing vehicle states, and for passing the generated constrained to the adaptive MPC controller.

In order to run the simulation, kindly follow these steps:
* Double-click `mpc_obstacle_avoidance.m` to load the model and workspace.
* Run the file. The `.slx` file will open and close automatically - this is expected behaviour and is part of the simulation. After the Simulink window closes, you will be able to view a plot detailing the obstacle avoidance response.