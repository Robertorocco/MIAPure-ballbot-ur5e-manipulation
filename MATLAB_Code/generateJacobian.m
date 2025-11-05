
% This script symbolically derives the BallBot's Jacobian and generates
% an optimized MATLAB function file 'calculateJacobian.m'.
% Run this script once to create the function.

clear;
clc;

fprintf('Generating Jacobian function...\n');

% 1. Define symbolic variables
syms theta_x theta_y phi_x phi_y R h real

% Define the vector of generalized coordinates (must match the order in Step 1)
q = [theta_x; theta_y; phi_x; phi_y];

% 2. Write the forward kinematics equations
% Position of the sphere's center
p_sphere_center = [R*phi_y; -R*phi_x; R];

% Rotation matrices for body tilt (Pitch around Y, then Roll around X)
Ry = [cos(theta_y), 0, sin(theta_y); 0, 1, 0; -sin(theta_y), 0, cos(theta_y)];
Rx = [1, 0, 0; 0, cos(theta_x), -sin(theta_x); 0, sin(theta_x), cos(theta_x)];
R_body = Rx * Ry;

% Local vector from sphere center to attachment point in the body frame
p_local = [0; 0; h];

% Final position of the attachment point in the world frame
p_attach = p_sphere_center + R_body * p_local;

% 3. Calculate the linear part of the Jacobian (Jp)
% This takes the partial derivative of p_attach with respect to each element of q
Jp = jacobian(p_attach, q);

% 4. Calculate the angular part of the Jacobian (Jo)
% The body's angular velocity omega is related to dot_q.
% omega = [dot_theta_x; 0; 0] + Rx * [0; dot_theta_y; 0]
% We extract the matrix that maps [dot_theta_x, dot_theta_y] to omega.
% The rolling rates dot_phi_x and dot_phi_y do not cause body rotation.
E = [1, sin(theta_x)*sin(theta_y)/cos(theta_y); % This can be simplified with assumptions
     0, cos(theta_x);
     0, sin(theta_x)/cos(theta_y)];
% Let's use a more robust formulation for angular velocity
omega_x_contrib = [1; 0; 0] * [1, 0, 0, 0]; % Contribution from dot_theta_x
omega_y_contrib = Rx * [0; 1; 0] * [0, 1, 0, 0]; % Contribution from dot_theta_y
Jo = omega_x_contrib + omega_y_contrib;


% 5. Assemble the full Jacobian matrix
J_bot = [Jp; Jo];

% 6. Generate an optimized MATLAB function from the symbolic expression
% The function will be saved as 'calculateJacobian.m'
fprintf('Creating MATLAB function file: calculateJacobian.m\n');
matlabFunction(J_bot, 'File', 'calculateJacobian', 'Vars', {q, R, h});

fprintf('Done!\n');