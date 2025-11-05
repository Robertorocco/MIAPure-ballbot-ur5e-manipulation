%% SCRIPT TO SIMULATE INNER-OUTER LOOP CONTROL OF THE BALLBOT
% =========================================================================
% This script implements a cascaded controller for position tracking.
%   - Outer Loop (PID): Calculates a desired velocity to reach a target.
%   - Inner Loop (LQR): Stabilizes the robot and tracks the desired velocity.
%
% It uses all the modular functions we have developed.
% =========================================================================

clear; clc; close all;
addpath('function');
%% PART 1: SYSTEM PARAMETERS
% -------------------------------------------------------------------------
ur5e_mass = 20.7;
robotiq_gripper_mass = 1;

p.h_U = 0.50;      % m (Sphere and Chassis estimated height)
p.m_S = 3.6;       % kg (Sphere mass)
p.I_S = 0.047;     % kgm^2 (Sphere inertia)
p.r_S = 0.1145;    % m (Sphere radius)
p.m_C = 17.9 - p.m_S; %kg (Chassis mass)
p.COM_C = (p.h_U-2*p.r_S)/2+p.r_S;         % m (Chassis COM height) 
p.m_R = ur5e_mass + robotiq_gripper_mass;
p.COM_R= 0.7649;          % m (Manipulator COM height)
p.m_U = p.m_C + p.m_R;      % kg (Upper body mass)
p.I_Ux = 0.1848 + 14.4706;      % kgm^2  Upper body Roll inertia)
p.I_Uy = 0.1848 + 15.0733;      % kgm^2 (Upper body Pitch inertia)
p.Iz = 0.0085 + 1.2023;        % kgm^2 (Upper body Yaw inertia)
p.l_U = (p.m_C * p.COM_C + p.m_R * p.COM_R) / p.m_U;      % m (Upper body COM height) (Include manipulator effect)
p.g = 9.81;        % m/s^2 (Gravitational acceleration)
p.b_theta = 0.0;   % Nms/rad (Viscous friction on tilt)
p.b_phi = 1;     % Nms/rad (Viscous friction on sphere rolling)
p.fz = 1;        % Nms/rad (Viscous friction on yaw)
p.r_ow=0.0625;     % m (ow radius)
using_robust= 0;
% Calculate the distance between the center of the sphere and the force's application point
p.h_app = p.h_U - p.r_S; % Distance from the center of the sphere to the force application point
% =========================================================================
% LQR CONTROLLER DESIGN (INNER LOOP)
% =========================================================================
disp('Designing LQR controller for inner loop...');

% --- Step 1: Linearize the Dynamic Model ---
x_eq = zeros(10, 1);
u_eq = zeros(3, 1);
[A, B] = linearizeBallbotModel(p, x_eq, [u_eq;zeros(5,1)]);

% --- Step 2: Decouple the System for LQR Design ---
pitch_states_idx = [2, 5, 7, 10]; % [theta_y; phi_y; dot_theta_y; dot_phi_y]
pitch_input_idx = 2; % tau_y
A_pitch = A(pitch_states_idx, pitch_states_idx);
B_pitch = B(pitch_states_idx, pitch_input_idx);

% --- Step 3: Design the LQR Controller ---
Q = diag([5, 0.1, 1, 1]); 
R = 0.1;
K = lqr(A_pitch, B_pitch, Q, R);
disp('LQR Gain Matrix K:');
disp(K);

% =========================================================================
% SIMULATION SETUP
% =========================================================================
% --- Outer Loop Controller Parameters ---
control_params.Kp_pos = 1.0;
control_params.Ki_pos = 0.2; % Integral gain to remove steady-state error
control_params.Kd_pos = 0.5; % Derivative gain to reduce overshoot
control_params.K = K; % Pass the LQR gains to the controller
control_params.p_target = [0.5; 0.3]; % Target: x=0.5m, y=0.3m

% --- Initial Conditions (Augmented State) ---
% The state vector is now augmented to include the integral of position error
% x = [theta_x, theta_y, theta_z, phi_x, phi_y, ...velocities..., int_err_x, int_err_y]
x0 = zeros(12, 1); 
t_span = [0 15];

% --- Run the Simulation ---
fprintf('\nRunning cascaded control simulation...\n');
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-5);
[t, x] = ode45(@(t,x) stateDerivative3D_Cascaded(t, x, p, control_params), t_span, x0, options);
fprintf('Simulation complete.\n');

% =========================================================================
% POST-PROCESSING AND PLOTTING
% =========================================================================
% --- Recalculate Controller and Torque History for Plotting ---
tau_history = zeros(length(t), 3);
internal_vars_history = cell(length(t), 1);
for i = 1:length(t)
    [tau, internal_vars] = calculateControlTorques(x(i,:)', p, control_params);
    tau_history(i,:) = tau';
    internal_vars_history{i} = internal_vars;
end
num_time_steps = length(t);

% Pre-alloca la matrice di output per le coppie delle omni-wheel (formato N x 3)
tau_history_ow = zeros(num_time_steps, 3);

% Itera su ogni istante di tempo
for i = 1:num_time_steps
    % Estrai le coppie della sfera [tau_x; tau_y; tau_z] per l'istante corrente
    % e assicurati che sia un vettore colonna 3x1
    current_tau_S_col = tau_history(i, :)';

    % Converti le coppie della sfera in coppie dei motori delle omni-wheel.
    % La tua funzione T_3D_inv si aspetta un vettore colonna come input.
    current_tau_W_col = T_3D_inv(p.r_S, p.r_ow, current_tau_S_col);

    % Salva il risultato (un vettore colonna 3x1) come una riga nella nuova matrice
    tau_history_ow(i, :) = current_tau_W_col';
end
% --- Plot Controller-Specific Performance ---
% Extract the history of internal controller variables
error_pos_cell = cellfun(@(v) v.error_pos, internal_vars_history, 'UniformOutput', false);
error_pos_history = cell2mat(error_pos_cell'); % Corrected: Transpose cell array

v_des_cell = cellfun(@(v) v.v_des, internal_vars_history, 'UniformOutput', false);
v_des_history = cell2mat(v_des_cell'); % Corrected: Transpose cell array

ref_state_y_cell = cellfun(@(v) v.ref_state_y, internal_vars_history, 'UniformOutput', false);
ref_state_y_history = cell2mat(ref_state_y_cell'); % Corrected: Transpose cell array

figure('Name', 'Controller Performance', 'NumberTitle', 'off');
sgtitle('Controller Performance Analysis');

% Subplot 1: Position Error (Outer Loop)
subplot(2,2,1);
plot(t, error_pos_history(1,:), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, error_pos_history(2,:), 'g-', 'LineWidth', 1.5);
grid on; title('Outer Loop: Position Error'); xlabel('Time (s)'); ylabel('Error (m)'); legend('Error X', 'Error Y');

% Subplot 2: Desired Velocity (Outer Loop Output)
subplot(2,2,2);
plot(t, v_des_history(1,:), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, v_des_history(2,:), 'g-', 'LineWidth', 1.5);
grid on; title('Outer Loop Output: Desired Velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend('v_{des, x}', 'v_{des, y}');

% Subplot 3: Velocity Tracking (Inner Loop)
subplot(2,2,3);
plot(t, ref_state_y_history(4,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x(:,10), 'r--', 'LineWidth', 1.5); % Actual phi_dot_y
grid on; title('Inner Loop Tracking (Pitch Velocity)'); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); legend('Desired d\phi_y/dt', 'Actual d\phi_y/dt', 'Interpreter', 'tex');

% --- Call Generic Plotting and Animation Functions ---
myplot(t, x, p, tau_history,tau_history_ow);
animateBallbot(t, x, p);


% =========================================================================
% NESTED FUNCTIONS FOR SIMULATION
% =========================================================================

function dx = stateDerivative3D_Cascaded(t, x, p, control_params)
    % This is the main function passed to ode45.
    % It calculates the control torques and then finds the resulting
    % state derivatives using our robust dynamics function.

    % --- 1. Calculate Control Torques ---
    [tau_vector, internal_vars] = calculateControlTorques(x, p, control_params);

    % --- 2. Calculate System Dynamics ---
    % Call the core, auto-generated dynamics function with the calculated torques
    dx_dynamics = stateDerivative3D_generated( x, p, tau_vector);
    
    % --- 3. Calculate Derivatives of Integrator States ---
    % The derivative of the integral of the error is the error itself.
    error_pos = internal_vars.error_pos;
    
    % --- 4. Assemble the final 12x1 state derivative vector ---
    dx = [
        dx_dynamics;    % First 10 derivatives from the core dynamics
        error_pos(1);   % d/dt(integral(error_x)) = error_x
        error_pos(2);   % d/dt(integral(error_y)) = error_y
    ];
end

function [tau, internal_vars] = calculateControlTorques(x, p, control)
    % This function contains all the cascaded control logic.
    
    % Unpack controller parameters and state vector (12 elements)
    K = control.K; Kp_pos = control.Kp_pos; Ki_pos = control.Ki_pos; Kd_pos = control.Kd_pos; p_target = control.p_target;
    theta_x = x(1); dot_theta_x = x(6);
    theta_y = x(2); dot_theta_y = x(7);
    phi_x   = x(4); dot_phi_x   = x(9);
    phi_y   = x(5); dot_phi_y   = x(10);
    int_err_x = x(11); int_err_y = x(12);

    % --- OUTER LOOP (PID Position Controller) ---
    p_current = [p.r_S * phi_y; -p.r_S * phi_x];
    error_pos = p_target - p_current;
    error_pos_int = [int_err_x; int_err_y];
    v_current = [p.r_S * dot_phi_y; -p.r_S * dot_phi_x];
    error_pos_dot = -v_current;
    v_des = (Kp_pos * error_pos) + (Ki_pos * error_pos_int) + (Kd_pos * error_pos_dot);
    phi_dot_des_y = v_des(1) / p.r_S;
    phi_dot_des_x = -v_des(2) / p.r_S; % Note the sign correction

    % --- INNER LOOP (LQR Stabilizer & Velocity Tracker) ---
    % For the Y-Z (Pitch) plane
    state_y = [theta_y; phi_y; dot_theta_y; dot_phi_y];
    ref_state_y = [0; phi_y; 0; phi_dot_des_y]; % Track desired velocity
    error_state_y = state_y - ref_state_y;
    tau_y = -K * error_state_y;
    % For the X-Z (Roll) plane
    state_x = [theta_x; phi_x; dot_theta_x; dot_phi_x];
    ref_state_x = [0; phi_x; 0; phi_dot_des_x]; % Track desired velocity
    error_state_x = state_x - ref_state_x;
    tau_x = -K * error_state_x;
    
    tau_z = 0; % Yaw not controlled
    
    tau = [tau_x; tau_y; tau_z];
    
    % Pack internal variables for logging
    internal_vars.error_pos = error_pos;
    internal_vars.v_des = v_des;
    internal_vars.ref_state_y = ref_state_y;
end
