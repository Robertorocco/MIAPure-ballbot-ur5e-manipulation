%% SCRIPT TO SIMULATE LQR STABILIZATION OF THE BALLBOT
% =========================================================================
% This script designs an LQR controller to stabilize the ballbot and then
% simulates the closed-loop system's response to an initial disturbance.
%
% This version is decoupled: it calculates the full torque history and
% passes it to a generic plotting function.
% =========================================================================

clear; clc; close all;
addpath('function');
% --- Numerical Parameters ---
p.m_U = 92;      % kg (Upper body mass)
p.I_Ux = 3.0;      % kgm^2 (Roll inertia)
p.I_Uy = 3.0;      % kgm^2 (Pitch inertia)
p.Iz = 1.0;        % kgm^2 (Yaw inertia)
p.l_U = 0.45;      % m (Upper body COM height)
p.m_S = 4;       % kg (Sphere mass)
p.I_S = 0.052;     % kgm^2 (Sphere inertia)
p.r_S = 0.1143;    % m (Sphere radius)
p.g = 9.81;        % m/s^2 (Gravitational acceleration)
p.b_theta = 0.0;   % Nms/rad (Viscous friction on tilt)
p.b_phi = 1;     % Nms/rad (Viscous friction on sphere rolling)
p.fz = 1;        % Nms/rad (Viscous friction on yaw)
p.r_ow=0.0625;     % m (ow radius)


% --- Initial Conditions ---
t_span = [0 10]; % Simulate for 10 seconds
x0 = zeros(10, 1);
x0(1) = 30 * (pi/180);  % Initial  tilt in X (Roll)
x0(2) = 0 * (pi/180);  % Initial  tilt in Y (Pitch)

% =========================================================================
% LQR CONTROLLER DESIGN
% =========================================================================
disp('Designing LQR controller...');

% --- Step 1: Linearize the Dynamic Model ---
x_eq = zeros(10, 1);
u_eq = zeros(3, 1);
[A, B] = linearizeBallbotModel(p, x_eq, u_eq);

% --- Step 2: Decouple the System for LQR Design ---
pitch_states_idx = [2, 5, 7, 10]; % [theta_y; phi_y; dot_theta_y; dot_phi_y]
pitch_input_idx = 2; % tau_y
A_pitch = A(pitch_states_idx, pitch_states_idx);
B_pitch = B(pitch_states_idx, pitch_input_idx);

% --- Step 3: Design the LQR Controller ---
Q = diag([10, 0.1, 1, 0.1]);
R = 0.1;
K = lqr(A_pitch, B_pitch, Q, R);
disp('LQR Gain Matrix K:');
disp(K);

% =========================================================================
% SIMULATION
% =========================================================================
fprintf('\nRunning closed-loop simulation...\n');

options = odeset('RelTol', 1e-5, 'AbsTol', 1e-5);
[t, x] = ode45(@(t,x) stateDerivative3D_LQR(x, p, K), t_span, x0, options);

fprintf('Simulation complete.\n');

% =========================================================================
% POST-PROCESSING AND PLOTTING
% =========================================================================
% --- Calculate Torque History ---
% This makes the plotting function generic.
tau_history = zeros(length(t), 3);
for i = 1:length(t)
    state_y = [x(i,2); x(i,5); x(i,7); x(i,10)];
    state_x = [x(i,1); x(i,4); x(i,6); x(i,9)];
    tau_history(i,1) = -K * state_x; % tau_x
    tau_history(i,2) = -K * state_y; % tau_y
    tau_history(i,3) = 0;            % tau_z
end

%Evaluate real torque applied by OW motors
tau_history_ow = zeros(size(tau_history));
for i = 1:size(tau_history, 1)
    % Get the current torque vector (as a row).
    current_tau_S_row = tau_history(i, :);
    
    % Your function expects a column vector, so we need to transpose the row.
    current_tau_S_col = current_tau_S_row'; 
    
    % Call the conversion function for the single column vector.
    current_tau_W_col = T_3D_inv(p.r_S, p.r_ow, current_tau_S_col);
    
    % The result is a column vector, so transpose it back to a row
    % and store it in the output matrix.
    tau_history_ow(i, :) = current_tau_W_col';
end

% --- Call the Generic Plotting and Animation Functions ---
myplot(t, x, p, tau_history,tau_history_ow);
animateBallbot(t, x, p);


% =========================================================================
% STATE DERIVATIVE FUNCTION FOR LQR SIMULATION
% =========================================================================
function dx = stateDerivative3D_LQR(x, p, K)
    % This function calculates the state derivative for the closed-loop
    % system under LQR control.

    % Unpack state vector
    theta_x = x(1); dot_theta_x = x(6);
    theta_y = x(2); dot_theta_y = x(7);
    phi_x   = x(4); dot_phi_x   = x(9);
    phi_y   = x(5); dot_phi_y   = x(10);
    
    % LQR CONTROL LAW: u = -Kx
    state_y = [theta_y; phi_y; dot_theta_y; dot_phi_y];
    tau_y = -K * state_y;
    state_x = [theta_x; phi_x; dot_theta_x; dot_phi_x];
    tau_x = -K * state_x;
    tau_z = 0;
    
    % Call the core dynamics function
    tau_inputs = [tau_x; tau_y; tau_z];
    dx = stateDerivative3D_generated(x, p, tau_inputs);
end
