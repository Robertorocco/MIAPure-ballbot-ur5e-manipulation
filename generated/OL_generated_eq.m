%% SCRIPT TO SIMULATE AND VISUALIZE THE 3D BALLBOT MODEL
% =========================================================================
% This script uses the auto-generated equations of motion to simulate the
% ballbot's behavior from a given initial condition and plots the results.
%
% It requires the following function files in the same directory:
%   - autogen_ddot_theta_x.m
%   - autogen_ddot_phi_x.m
%   - autogen_ddot_theta_y.m
%   - autogen_ddot_phi_y.m
% These are created by running the 'ballbot_model_generator.m' script.
% =========================================================================

% =========================================================================
% PART 1: SETUP
% =========================================================================
clear; clc; close all;
addpath('function');
% --- Numerical Parameters ---
% p.m_U = 92;      % kg (Upper body mass)
% p.I_Ux = 3.0;      % kgm^2 (Roll inertia)
% p.I_Uy = 3.0;      % kgm^2 (Pitch inertia)
% p.Iz = 1.0;        % kgm^2 (Yaw inertia)
% p.l_U = 0.45;      % m (Upper body COM height)
% p.m_S = 4;       % kg (Sphere mass)
% p.I_S = 0.052;     % kgm^2 (Sphere inertia)
% p.r_S = 0.1143;    % m (Sphere radius)
% p.g = 9.81;        % m/s^2 (Gravitational acceleration)
% p.b_theta = 0.0;   % Nms/rad (Viscous friction on tilt)
% p.b_phi = 1;     % Nms/rad (Viscous friction on sphere rolling)
% p.fz = 1;        % Nms/rad (Viscous friction on yaw)
% p.r_ow=0.0625;     % m (ow radius)

p.h_U = 0.50;      % m (Sphere and Chassis estimated height)
p.m_S = 3.6;       % kg (Sphere mass)
p.I_S = 0.047;     % kgm^2 (Sphere inertia)
p.r_S = 0.1145;    % m (Sphere radius)
p.m_C = 17.9 - p.m_S; %kg (Chassis mass)
p.COM_C = (p.h_U-2*p.r_S)/2+p.r_S;         % m (Chassis COM height) 
p.m_U = p.m_C      % kg (Upper body mass)
p.I_Ux = 0.1848;     % kgm^2  Upper body Roll inertia)
p.I_Uy = 0.1848;      % kgm^2 (Upper body Pitch inertia)
p.Iz = 0.0085;        % kgm^2 (Upper body Yaw inertia)
p.l_U = p.COM_C;      % m (Upper body COM height) (Include manipulator effect)
p.g = 9.81;        % m/s^2 (Gravitational acceleration)
p.b_theta = 0.0;   % Nms/rad (Viscous friction on tilt)
p.b_phi = 1;     % Nms/rad (Viscous friction on sphere rolling)
p.fz = 1;        % Nms/rad (Viscous friction on yaw)
p.r_ow=0.0625;     % m (ow radius)
p.h_app = p.h_U - p.r_S; % Distance from the center of the sphere to the force application point


% --- Initial Conditions and Inputs ---
t_span = [0 10]; % Simulate for 10 seconds
% State vector: [theta_x, theta_y, theta_z, phi_x, phi_y, ...derivatives]
x0 = zeros(10, 1);
x0(1) = 5 * (pi/180);  % Initial tilt in X (Roll) of 5 degrees
x0(2) = 0 * (pi/180);  % Initial tilt in Y (Pitch)

% Input torques (currently zero to observe the free fall)
tau_inputs = [0; 0; 0]; % [tau_x; tau_y; tau_z]

% --- Run the Simulation ---
fprintf('Running simulation with the corrected model...\n');
% Use ode45 with the function describing the 3D dynamics
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, x] = ode45(@(t,x) stateDerivative3D_generated(x, p, tau_inputs), t_span, x0, options);
tau_inputs = zeros(length(t),3);
fprintf('Simulation complete.\n');
tau_ow = zeros(size(tau_inputs));

% =========================================================================
% PART 2: PLOT
% =========================================================================

 myplot(t, x, p, tau_inputs,tau_ow)
 % =========================================================================
% PART 3: 3D ANIMATION
% =========================================================================
 animateBallbot(t, x, p)


