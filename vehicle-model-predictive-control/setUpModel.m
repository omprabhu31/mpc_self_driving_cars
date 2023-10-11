% Model InitialI_zation
% MPC Controller for Figure Eight Course

%% add Images to the path
addpath(genpath('Images'));

%% define reference points
r = 75; % radius of circle
theta = 0:0.01:2 * pi;

x_1 = -r * cos(theta);
y_1 = r * sin(theta);
x_2 = -2 * r + r * cos(theta);
y_2 = r * sin(theta);

x_eight = [x_1 x_1 x_2 x_2];
y_eight = [y_1 y_1 y_2 y_2];

x_ref = y_eight;
y_ref = x_eight;

%% define vehicle parameters used in the models
L = 3; % bicycle length
l_d = 4; % lookahead distance
X_o = x_ref(1); % initial vehicle position
Y_o = y_ref(1); % initial vehicle position 
psi_o = 0; % initialI_ze yaw angle

%% calculating reference pose vectors for figure eight course

% calculate distance vector
rr = [y_eight; x_eight; x_eight * 0]';
distance_matrix = squareform(pdist(rr));
distance_steps = zeros(length(rr) - 1, 1);

for i = 2:length(rr)
    distance_steps(i - 1, 1) = distance_matrix(i, i - 1);
end

total_distance = sum(distance_steps);         % total traveled distance
dist = cumsum([0; distance_steps]);        % distance for each waypoint
grad = linspace(0, total_distance, 2500);    % linearI_ze distance

% linearI_ze X and Y vectors based on distance
x_ref2 = interp1(dist, x_ref, grad, 'pchip');
y_ref2 = interp1(dist, y_ref, grad, 'pchip');
y_ref2s = smooth(grad, y_ref2);
x_ref2s = smooth(grad, x_ref2);

% calculate curvature vector
curvature = getCurvature(x_ref2, y_ref2);

%% Calculate A, B, C matrices used in MPC Model
% vehicle parameters
tau = 0.5;
V_x = 10;
m = 2000;
I_z = 4000;
L_f = 1.4;
Lr = 1.6;
C_f = 12e3;
C_r = 11e3;

% longitudinal model
A_1 = [-1/tau 0; 1 0];
B_1 = [1/tau; 0];
C_1 = [ 0 1];
D_1 = 0;

% lateral model
A_2 = [-2 * (C_f + C_r) / (V_x * m) -V_x - 2 * (C_f * L_f-C_r * L_r) / (V_x * m);...
    -2 * (C_f * L_f-C_r * Lr) / (V_x * I_z) -2 * (C_f * L_f^2 + C_r * Lr^2) / (V_x * I_z)];
B_2 = [2 * C_f/m; 2 * C_f * L_f/I_z];
C_2 = [1 0; 0 1];
D_2 = [0;0];

% combined model
A = [A_1 zeros(2, 2); zeros(2, 2) A_2];
B = [B_1 zeros(2, 1); zeros(2, 1) B_2];
C = [C_1 zeros(1, 2); zeros(2, 2) C_2];
D = [D_1 zeros(1, 1); zeros(2, 1) D_2];

%% MPC Pedal Map
% additional vehicle parameters
rho = 1.21;
C_d = 0.3;
A_f = 2;
tire_r = 0.309;

% bounds for 2-D lookup table
accel_vec = (-4:0.5:4)'; % acceleration is between -4 and 4 m/s^2
vel_vec = 0:2:20; % vehicle velocity is between 0 and 20 m/s
torque_map = zeros(length(accel_vec), length(vel_vec));

% calculate required torque
for i = 1:length(accel_vec)
    for j = 1:length(vel_vec)
        % torque = tire force + resistive force
        torque_map(i, j) = tire_r * ((m * accel_vec(i)) + (0.5 * rho * C_d * A_f * vel_vec(j)^2) + 160);
    end
end

% convert torque to pedal based on powertrain parameters
pedal_map = torque_map;

% positive torques are scaled based on powertrain's maximum wheel torque
max_prop_torque = 425 * 9.5;
pedal_map(pedal_map>0) = pedal_map(pedal_map>0)/max_prop_torque;

% calculate the conversion from torque to maximum pressure
pressure_conv = (0.2 * 7.5e6 * pi * 0.05 * 0.05 * .177 * 2/4) * 4 * 1.01; % 1.01 is a calibrated value
pedal_map(pedal_map<0) = pedal_map(pedal_map<0)/pressure_conv;

%% Curvature Function

function curvature = getCurvature(x_ref, y_ref)
% calculate curvature function using differential X and Y vectors
DX = gradient(x_ref);
D_2X = gradient(DX);
DY = gradient(y_ref);
D_2Y = gradient(DY);
curvature = (DX.* D_2Y - DY.* D_2X) ./(DX.^2 + DY.^2).^(3/2);
end
