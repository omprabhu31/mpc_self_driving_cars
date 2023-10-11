function [x_dot, y] = ct_vehicle_model(x, u)
% vehicle parameters
car_length = 5;
theta = x(3);
V = x(4);
delta = u(2);

% compute CT model matrices
A = [ 0, 0, -V * sin(theta), cos(theta);
      0, 0,  V * cos(theta), sin(theta);
      0, 0, 0,             tan(delta)/car_length;
      0, 0, 0,             0];
B = [0  , 0;
     0  , 0;
     0  , (V * (tan(delta)^2 + 1))/car_length;
     0.5, 0];

C = eye(4);
D = zeros(4,2);

x_dot = A * x + B * u;
y = C * x + D * u;