function [A_d, B_d, C_d, D_d, U, Y, X, DX] = dt_vehicle_model(T_s, x, u)
% define CT linear model from Jacobian of the nonlinear state-space model
car_length = 5;
theta = x(3);
V = x(4);
delta = u(2);

A_c = [ 0, 0, -V * sin(theta), cos(theta);
       0, 0,  V * cos(theta), sin(theta);
       0, 0, 0,             tan(delta)/car_length;
       0, 0, 0,             0];

B_c = [0  , 0;
      0  , 0;
      0  , (V * (tan(delta)^2 + 1))/car_length;
      0.5, 0];

C_c = eye(4);
D_c = zeros(4,2);

% generate discrete-time model using zero-order method
[A_d, B_d] = adasblocks_utilDicretizeModel(A_c, B_c, T_s);
C_d = C_c;
D_d = D_c;

% compute nominal conditions for discrete-time plant
X = x;
U = u;
Y = x;
DX = A_d * x + B_d * u - x;