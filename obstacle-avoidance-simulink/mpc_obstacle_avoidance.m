%% Obstacle Avoidance using MPC
% construct linear prediction model using using Jacobians of the
% vehicle state-space model
V = 30;
x0 = [0; 0; 0; V];
u_0 = [0; 0];

% discretize the CT model using the dt_vehicle_model function
Ts = 0.02;
[A_d, B_d, C_d, D_d, U, Y, X, DX] = dt_vehicle_model(Ts, x0, u_0);
dsys = ss(A_d, B_d, C_d, D_d, Ts = Ts);
dsys.InputName = {'Throttle','Delta'};
dsys.StateName = {'X','Y','Theta','V'};
dsys.OutputName = dsys.StateName;

%% Road and Obstacle Information
lanes = 3;
lane_width = 4;

% define obstacle parameters and detection distance
obstacle = struct;
obstacle.Length = 10;
obstacle.Width = 1;
obstacle.X = 50;
obstacle.Y = 0;
obstacle.DetectionDistance = 30;

% define safe zone around the obstacle
obstacle.safeDistanceX = obstacle.Length;
obstacle.safeDistanceY = lane_width;
obstacle = define_obstacle(obstacle);

%% Initialize the results plot
f = initialize_plot(x0, obstacle, lane_width, lanes);

%% MPC Design
status = mpcverbosity("off");
mpcobj = mpc(dsys);

% define prediction and control horizon for the MPC algorithm
mpcobj.PredictionHorizon = 60;
mpcobj.ControlHorizon = 5;

% define hard constraints on vehicle acceleration for safety
mpcobj.ManipulatedVariables(1).RateMin = -0.2 * Ts; 
mpcobj.ManipulatedVariables(1).RateMax = 0.2 * Ts;

% define hard constraints on yaw rate to avoid jerky steering action
mpcobj.ManipulatedVariables(2).RateMin = -pi/30 * Ts;
mpcobj.ManipulatedVariables(2).RateMax = pi/30 * Ts;

% define weights of control variables
mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;
mpcobj.Weights.OutputVariables = [0 30 0 1];

%% Update MPC controller with discretized plant inputs
mpcobj.Model.Nominal = struct(U=U, Y=Y, X=X, DX=DX);

%% Constraints for Obstacle Avoidance 
% upper and lower bounds on y-coordinate (depending on number of lanes)
E_1 = [0 0];
F_1 = [0 1 0 0]; 
G_1 = lane_width * lanes/2;

E_2 = [0 0];
F_2 = [0 -1 0 0]; 
G_2 = lane_width * lanes/2;

% placeholder constraint matrices for future obstacle constraint generation
E_3 = [0 0];
F_3 = [0 -1 0 0]; 
G_3 = lane_width * lanes/2;

setconstraint(mpcobj, [E_1;E_2;E_3], [F_1;F_2;F_3], [G_1;G_2;G_3], [1;1;0.1]);

%% Simulation of the MPC Controller
refSignal = [0 0 0 V];

% initialize plant states, controller states and sample time
x = x0;
u = u_0;
ego_states = mpcstate(mpcobj);
T = 0:Ts:4;

% log simulation data for plotting

save_slope = zeros(length(T),1);
save_y_intercept = zeros(length(T),1);
ympc = zeros(length(T),size(C_d,1));
umpc = zeros(length(T),size(B_d,2));

for k = 1:length(T)
    % obtain new plant model for k-th time step
    [A_d, B_d, C_d, D_d, U, Y, X, DX] = dt_vehicle_model(Ts, x, u);
    measurements = C_d * x + D_d * u;
    ympc(k,:) = measurements';
    
    % update constraints based on whether an obstacle is detected or not
    detection = detect_obstacle(x, obstacle, lane_width);
    [E, F, G, save_slope(k), save_y_intercept(k)] = compute_obstacle_constraints(x, detection, obstacle, lane_width, lanes); 
   
    % compute new model and constraints
    new_plant = ss(A_d,B_d,C_d,D_d,Ts=Ts);
    new_nominal_condition = struct(U=U,Y=Y,X=X,DX=DX);

    % compute the optimal control and update the model
    options = mpcmoveopt;
    options.CustomConstraint = struct(E=E,F=F,G=G);
    [u,Info] = mpcmoveAdaptive(mpcobj, ego_states, new_plant, new_nominal_condition, measurements, refSignal, [], options);
    umpc(k,:) = u';
    
    % set state for (k+1)-th time step
    x = A_d * x + B_d * u;
end

mpcverbosity(status);

%% Analysis of Simulation Results
% black plot: vehicle trajectory
% green dashed lines: constraints (at any given time step, the vehicle
% cannot enter the region on the right of the respective dashed line)

figure(f)
for k = 1:length(save_slope)
    X = [0;50;100];
    Y = save_slope(k) * X + save_y_intercept(k);
    line(X, Y, LineStyle="--", Color="cyan")
end    
plot(ympc(:,1), ympc(:,2), "-k");
axis([0 ympc(end,1) -lane_width * lanes/2 lane_width * lanes/2]) % reset axis

%% Run the Controller Model in Simulink

simulink_model = "obstacle_avoidance_mpc";
open_system(simulink_model)
sim(simulink_model)
bdclose(simulink_model)
