# Basic MPC Control for Trajectory Tracking
### Part of  Dual Degree Project - Stage I

This folder contains the relevant code files for simulation of a basic MPC controller for autonomous vehicle path tracking. The simulation is carried out using Simulink, and you will need the version of Simulink supplied with MATLAB R2023b in order to run the files.

Certain function blocks pertaining to the simulation have been adapted from the [MATLAB Central File Exchange](https://www.mathworks.com/matlabcentral/fileexchange/68992-designing-an-mpc-controller-with-simulink), namely the vehicle dynamics computation functionality offered by the `Meldas_library.slx` file.

In order to run the simulation, kindly follow these steps:
* Double-click `basic_mpc_controller.slx` to load the model and workspace. Do NOT run the simulation at this stage.
* Double-click `Params.mat` to load relevant parameters.
* Right-click on `ReferenceGenerator.m` in your MATLAB worksace and select 'Run'. This will simulate the driving scenario and store the reference pose vectors in the workspace.
* Run the simulation. Use the display blocks to access the plots for Lateral Position and Yaw Angle.