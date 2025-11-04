%% SCRIPT TO SIMULATE CASCADED LQR-PI CONTROLLER (Corrected Implementation)
% =========================================================================
% This script implements the control architecture from the paper
% "PERSONAL UNIQUE ROLLING EXPERIENCE", Section 2.5.1.
%
% Architecture as per the paper:
% 1. Outer-most Loop (P-Controller): Converts a position error into a
%    *commanded* velocity for the LQR controller.
% 2. LQR Loop: Calculates a reference feedforward torque (tau_r) based on
%    the commanded velocity and the goal of keeping the robot upright.
% 3. Reference Model: Uses the reference torque (tau_r) and the linearized
%    system dynamics to calculate a reference wheel acceleration (ddot_phi_r).
% 4. Integration: The reference acceleration is integrated by the ODE solver
%    to produce a dynamic reference velocity (dot_phi_r).
% 5. Inner PI Loop: Ensures the actual wheel velocity tracks the reference
%    velocity by calculating a correction/tracking torque (tau_e).
% 6. Total Torque: The final torque is the sum of the LQR's reference
%    torque and the PI's tracking torque (tau_r + tau_e).
% =========================================================================
clear; clc; close all;
% Add the path to any necessary helper functions (e.g., for animation)
addpath('function');

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

uncertainty_perc = 20; % Define the uncertainty percentage (e.g., 10%)
p_plant = add_uncertainties(p, uncertainty_perc); % Create uncertain parameters for the simulation

% =========================================================================
% LQR DESIGN
% =========================================================================
disp('Designing LQR gain K...');

% --- Step 1: Linearize the full 3D dynamic model at the upright position ---
% Assuming a function 'linearizeBallbotModel' exists that returns the
% state-space matrices for the full 10-state model.
x_eq = zeros(10, 1);
u_eq = zeros(3, 1);
% --- IMPORTANT: Use the NOMINAL 'p' for linearization ---
[A_full, B_full] = linearizeBallbotModel(p, x_eq, [u_eq;zeros(5,1)]);

% --- Step 2: Decouple the model for a single planar controller (Pitch/Roll) ---
% The controller for the X and Y axes will be identical.
pitch_states_idx = [2, 5, 7, 10]; % [theta_y; phi_y; dot_theta_y; dot_phi_y]
pitch_input_idx = 2; % tau_y
A = A_full(pitch_states_idx, pitch_states_idx);
B = B_full(pitch_states_idx, pitch_input_idx);
disp('Linearized System Matrix A (for one plane):');
disp(A);
disp('Linearized System Matrix B (for one plane):');
disp(B);

% --- Step 3: Design the LQR Controller ---
% Weights for the state vector [theta, phi, dot_theta, dot_phi] and input u
Q = diag([5, 0.1, 1, 1]); 
R = 0.1;
[K, S, e] = lqr(A, B, Q, R);
disp('LQR Gain Matrix K:');
disp(K);

% =========================================================================
% SIMULATION SETUP
% =========================================================================
% State vector is augmented for the controller.
% x(1:10)  -> Robot dynamic states [theta_x, theta_y, psi, phi_x, phi_y, dot_theta_x, ...]
% x(11:12) -> Integral of speed tracking error [int_err_vx, int_err_vy]
% x(13:14) -> Reference wheel speeds [dot_phi_rx, dot_phi_ry]
x0 = zeros(14, 1);
t_span = [0 10];

% --- Define Controller Gains and Parameters ---
control_params.p_plant = p_plant; % Store uncertain 'p' (might not be needed by controller itself)
control_params.p_nominal = p;
control_params.K = K;           % LQR gain
control_params.A_lqr = A;       % Linearized model A matrix for reference acceleration calculation
control_params.B_lqr = B;       % Linearized model B matrix
control_params.p_target = [0.5; 0.3]; % Target position: x=0.5m, y=0.3m

% Proportional gain for outer position loop
control_params.Kp_pos = 0.5; %It influences the time taken to travel

%Inner PI speed tracking loop
control_params.Kp_inner = 1;  % Proportional gain  
control_params.Ki_inner = 0.5;  % Integral gain 

% control_params.Kp_inner = 0.0;  % Proportional gain  
% control_params.Ki_inner = 0.0;  % Integral gain 

% --- Run the Simulation ---
fprintf('\nRunning Cascaded LQR-PI control simulation...\n');
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-5);
[t, x] = ode45(@(t,x) stateDerivative_Cascaded(t, x, p_plant, control_params), t_span, x0, options);
fprintf('Simulation complete.\n');

% =========================================================================
% POST-PROCESSING & PLOTTING
% =========================================================================
% --- Recalculate internal controller variables over time for analysis ---
num_steps = length(t);
tau_r_hist = zeros(2, num_steps);
tau_e_hist = zeros(2, num_steps);
for i = 1:num_steps
    % We call the control function again for each time step to get the
    % internal torque values for plotting.
    [tau_total_i, internals] = calculate_Cascaded_Torques(x(i,:)', p, control_params);
    tau_r_hist(:, i) = internals.tau_r;
    tau_e_hist(:, i) = internals.tau_e;
    tau_total(i,:)=tau_total_i;
end
% =========================================================================
% PERFORMANCE METRIC CALCULATION (Velocity Tracking RMSE)
% =========================================================================
fprintf('\nCalculating performance metrics...\n');

% Extract actual and reference velocities
dot_phi_x_actual = x(:,9);
dot_phi_y_actual = x(:,10);
dot_phi_x_ref    = x(:,13);
dot_phi_y_ref    = x(:,14);

% Calculate squared errors at each time step
squared_error_vx = (dot_phi_x_ref - dot_phi_x_actual).^2;
squared_error_vy = (dot_phi_y_ref - dot_phi_y_actual).^2;
total_squared_error = squared_error_vx + squared_error_vy;

% Calculate the mean squared error (approximating integral with mean)
mean_squared_error = mean(total_squared_error);

% Calculate the Root Mean Square Error (RMSE)
velocity_tracking_rmse = sqrt(mean_squared_error);

% --- Optional: Plot Velocity Errors Over Time ---
figure('Name', 'Velocity Tracking Errors', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, dot_phi_y_ref - dot_phi_y_actual, 'b-', 'LineWidth', 1.5);
grid on;
title('Velocity Tracking Error (Y-axis / Pitch)');
xlabel('Time (s)');
ylabel('Error (rad/s)');
legend('\phi_{ry} - \phi_y', 'Interpreter', 'tex');

subplot(2,1,2);
plot(t, dot_phi_x_ref - dot_phi_x_actual, 'r-', 'LineWidth', 1.5);
grid on;
title('Velocity Tracking Error (X-axis / Roll)');
xlabel('Time (s)');
ylabel('Error (rad/s)');
legend('\phi_{rx} - \phi_x', 'Interpreter', 'tex');


% --- Calculate Position Tracking RMSE ---

% Extract actual position history
pos_x_actual = p.r_S * x(:,5);
pos_y_actual = -p.r_S * x(:,4);

% Get target position
p_target_x = control_params.p_target(1);
p_target_y = control_params.p_target(2);

% Calculate squared position errors at each time step
squared_error_pos_x = (p_target_x - pos_x_actual).^2;
squared_error_pos_y = (p_target_y - pos_y_actual).^2;
total_squared_pos_error = squared_error_pos_x + squared_error_pos_y;

% Calculate the mean squared position error
mean_squared_pos_error = mean(total_squared_pos_error);
% Calculate the Position Root Mean Square Error (RMSE)
position_tracking_rmse = sqrt(mean_squared_pos_error);
fprintf('----------------------------------------\n');
fprintf('  Tracking Performance Metrics\n');
fprintf('----------------------------------------\n');
fprintf('Position Tracking RMSE:     %.4f meters\n', position_tracking_rmse);
fprintf('Velocity Tracking RMSE:     %.4f rad/s\n', velocity_tracking_rmse);
fprintf('----------------------------------------\n\n');


% Pre-alloca la matrice di output per le coppie delle omni-wheel (formato N x 3)
tau_history_ow = zeros(num_steps, 3);

% Itera su ogni istante di tempo
for i = 1:num_steps
    % Estrai le coppie della sfera [tau_x; tau_y; tau_z] per l'istante corrente
    % e assicurati che sia un vettore colonna 3x1
    current_tau_S_col = tau_total(i, :)';

    % Converti le coppie della sfera in coppie dei motori delle omni-wheel.
    % La tua funzione T_3D_inv si aspetta un vettore colonna come input.
    current_tau_W_col = T_3D_inv(p.r_S, p.r_ow, current_tau_S_col);

    % Salva il risultato (un vettore colonna 3x1) come una riga nella nuova matrice
    tau_history_ow(i, :) = current_tau_W_col';
end
% --- Recalculate Position Error History ---
% This is needed for the new subplot
pos_error_hist = zeros(2, num_steps);
for i = 1:num_steps
    phi_y_i = x(i, 5);
    phi_x_i = x(i, 4);
    p_current_i = [p.r_S * phi_y_i; -p.r_S * phi_x_i];
    pos_error_hist(:, i) = control_params.p_target - p_current_i;
end

% --- Define Professional Colors ---
color_ref = 'b';      % Blue for reference signals and primary states
color_actual = 'r';   % Red for actual signals or X-axis errors/signals
color_secondary = 'g';% Green for Y-axis errors/signals
color_tau_r = 'c';    % Cyan for reference torque
color_tau_e = 'm';    % Magenta for tracking torque
color_tau_total = 'k';% Black for total torque

% --- Figure 1: LQR-PI Controller Performance ---
figure('Name', 'Cascaded LQR-PI Controller Performance', 'NumberTitle', 'off', 'Color', 'w');
% --- Increased Font Size for the Main Title ---
%sgtitle('Cascaded LQR-PI Controller Analysis', 'FontSize', 16);

% --- Subplot 1: Inner Loop Velocity Tracking (Y-axis / Pitch) ---
subplot(2,2,1);
plot(t, x(:,14), 'Color', color_ref, 'LineStyle', '-', 'LineWidth', 2); % Reference velocity dot_phi_ry
hold on;
plot(t, x(:,10), 'Color', color_actual, 'LineStyle', '--', 'LineWidth', 1.5); % Actual velocity dot_phi_y
grid on;
% --- Increased Font Size for Title ---
title('Inner Loop: Velocity Tracking (Pitch)', 'FontSize', 14);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
% --- Updated Legend Position ---
legend('Reference \phi_y/dt', 'Actual \phi_y/dt', 'Interpreter', 'tex', 'Location', 'northeast', 'FontSize', 12);
hold off;

% --- Subplot 2: Torque Contribution (Y-axis / Pitch) ---
subplot(2,2,2);
plot(t, tau_r_hist(2,:), 'Color', color_tau_r, 'LineStyle', '-', 'LineWidth', 1.5); % LQR Reference Torque (Pitch)
hold on;
plot(t, tau_e_hist(2,:), 'Color', color_tau_e, 'LineStyle', '-', 'LineWidth', 1.5); % PI Tracking Torque (Pitch)
plot(t, tau_r_hist(2,:) + tau_e_hist(2,:), 'Color', color_tau_total, 'LineStyle', '-', 'LineWidth', 2); % Total Torque (Pitch)
grid on;
% --- Increased Font Size for Title ---
title('Torque Contribution (Pitch)', 'FontSize', 14);
xlabel('Time (s)');
ylabel('Torque (Nm)');
% --- Increased Font Size for Legend ---
legend('\tau_{ref} (LQR)', '\tau_{track} (PI)', '\tau_{total}', 'Interpreter', 'tex', 'Location', 'best', 'FontSize', 12);
hold off;

% --- Subplot 3: Position Error ---
subplot(2,2,3);
plot(t, pos_error_hist(1,:), 'Color', color_actual, 'LineStyle', '-', 'LineWidth', 1.5); % Error in X position
hold on;
plot(t, pos_error_hist(2,:), 'Color', color_secondary, 'LineStyle', '-', 'LineWidth', 1.5); % Error in Y position
grid on;
% --- Increased Font Size for Title ---
title('Outer Loop: Position Error', 'FontSize', 14);
xlabel('Time (s)');
ylabel('Error (m)');
% --- Updated Legend Position ---
legend('Error X', 'Error Y', 'Location', 'northeast', 'FontSize', 12);
hold off;

% --- Subplot 4: Tilt Angle (Y-axis / Pitch) ---
subplot(2,2,4);
plot(t, rad2deg(x(:,2)), 'Color', color_ref, 'LineStyle', '-', 'LineWidth', 1.5); % Pitch angle theta_y
grid on;
% --- Increased Font Size for Title ---
title('Tilt Angle (Pitch)', 'FontSize', 14);
xlabel('Time (s)');
ylabel('Angle (degrees)');
% --- Increased Font Size for Legend ---
legend('\theta_y', 'Interpreter', 'tex', 'Location', 'best', 'FontSize', 12);
hold off;

% --- Optional: Adjust layout if needed ---
% Uncomment the line below if subplots overlap too much
% sgtitle('Cascaded LQR-PI Controller Analysis', 'FontSize', 14);
%myplot(t, x, p, tau_total,tau_history_ow);
%animateBallbot(t, x, p);

% =========================================================================
% CONTROLLER AND DYNAMICS FUNCTIONS
% =========================================================================

function dx = stateDerivative_Cascaded(t, x, p, control)
    % This is the main dynamics function for the ODE solver.
    % It calculates the derivative of the augmented 14x1 state vector.
    
    % --- 1. Calculate Control Torques & Reference Acceleration ---
    % This function now returns the total torque to be applied, and also
    % the reference acceleration needed to integrate the reference velocity state.

    % --- IMPORTANT: Controller calculations use NOMINAL parameters ---
    [tau_vector, internals] = calculate_Cascaded_Torques(x, control.p_nominal, control);
    
    % --- 2. Calculate System Dynamics (for the first 10 states) ---
    % This function calculates the derivatives of the physical states of the robot.
    % It should be a function generated from your dynamic model.
    dx_dynamics = stateDerivative3D_generated(x(1:10), p, tau_vector);
    
    % --- 3. Calculate Derivatives for Controller States (states 11-14) ---
    % The derivative of the integral of speed error IS the speed error.
    d_int_err_vx = internals.speed_error(1); % (dot_phi_rx - dot_phi_x)
    d_int_err_vy = internals.speed_error(2); % (dot_phi_ry - dot_phi_y)
    
    % The derivative of the reference speed IS the reference acceleration.
    ddot_phi_r_x = internals.ddot_phi_r(1);
    ddot_phi_r_y = internals.ddot_phi_r(2);
    
    % --- 4. Assemble the full 14x1 state derivative vector 'dx' ---
    dx = [ 
        dx_dynamics;    % First 10 derivatives from robot dynamics
        d_int_err_vx;   % Derivative of x-velocity error integral
        d_int_err_vy;   % Derivative of y-velocity error integral
        ddot_phi_r_x;   % Derivative of reference x-velocity (i.e., ref accel)
        ddot_phi_r_y;   % Derivative of reference y-velocity (i.e., ref accel)
    ];
end

function [tau_total, internal_vars] = calculate_Cascaded_Torques(x, p, control)
    % Implements the cascaded LQR-PI control logic as described in the paper.
    
    % Unpack controller parameters
    K = control.K;
    A = control.A_lqr; % Linearized model A
    B = control.B_lqr; % Linearized model B
    Kp_pos = control.Kp_pos;
    Kp_inner = control.Kp_inner;
    Ki_inner = control.Ki_inner;
    p_target = control.p_target;
    
    % Unpack relevant states from the 14x1 state vector
    theta_x = x(1); dot_theta_x = x(6);
    theta_y = x(2); dot_theta_y = x(7);
    phi_x   = x(4); dot_phi_x   = x(9);
    phi_y   = x(5); dot_phi_y   = x(10);
    int_err_vx = x(11);
    int_err_vy = x(12);
    dot_phi_rx = x(13); % Reference velocity is now a state from the integrator
    dot_phi_ry = x(14); % Reference velocity is now a state from the integrator
    
    % --- STEP 1: OUTER-MOST LOOP (P Position Controller) ---
    % Converts position error into a *commanded* velocity for the LQR loop.
    p_current = [p.r_S * phi_y; -p.r_S * phi_x];
    error_pos = p_target - p_current;
    v_cmd = Kp_pos * error_pos; % Commanded velocity [vx; vy]
    
    % Convert commanded cartesian velocity to commanded wheel angular velocity
    dot_phi_cmd_y = v_cmd(1) / p.r_S;
    dot_phi_cmd_x = -v_cmd(2) / p.r_S;
    
    % --- The following logic is applied identically for both X and Y planes ---
    
    % -- Sagittal Plane (Y) --
    state_y = [theta_y; phi_y; dot_theta_y; dot_phi_y];
    % The command state for LQR: stay upright (theta=0, dot_theta=0) and
    % achieve the commanded speed from the position controller. phi_y is a feed-forward term.
    command_state_y = [0; phi_y; 0; dot_phi_cmd_y];
    
    % STEP 2: Eq (2.11) - Calculate reference torque from LQR
    tau_ry = -K * (state_y - command_state_y);
    
    % STEP 3: Eq (2.12) part 1 - Calculate reference *acceleration* using linearized EOM
    % This is the core of the reference model: ddot_phi = A(4,:)*state + B(4)*tau
    ddot_phi_ry = A(4,:) * state_y + B(4) * tau_ry;
    % This ddot_phi_ry will be integrated by the ODE solver to give dot_phi_ry (state x(14)).
    
    % STEP 4: Eq (2.13) - Calculate PI tracking torque
    % The speed error is between the integrated reference speed (a state) and the actual speed.
    speed_error_y = dot_phi_ry - dot_phi_y;
    tau_ey = Kp_inner * speed_error_y + Ki_inner * int_err_vy;
    
    % STEP 5: Eq (2.14) - Summation of torques for the Y plane
    tau_y = tau_ry + tau_ey;

    % -- Frontal Plane (X) --
    state_x = [theta_x; phi_x; dot_theta_x; dot_phi_x];
    command_state_x = [0; phi_x; 0; dot_phi_cmd_x];
    tau_rx = -K * (state_x - command_state_x);
    ddot_phi_rx = A(4,:) * state_x + B(4) * tau_rx;
    speed_error_x = dot_phi_rx - dot_phi_x;
    tau_ex = Kp_inner * speed_error_x + Ki_inner * int_err_vx;
    tau_x = tau_rx + tau_ex;
    
    % --- FINAL ASSEMBLY ---
    tau_z = 0; % Yaw is not controlled in this example
    tau_total = [tau_x; tau_y; tau_z];
    
    % Pack internal variables for logging and for use in the state derivative function
    internal_vars.tau_r = [tau_rx; tau_ry];
    internal_vars.tau_e = [tau_ex; tau_ey];
    internal_vars.ddot_phi_r = [ddot_phi_rx; ddot_phi_ry]; % Reference acceleration
    internal_vars.speed_error = [speed_error_x; speed_error_y]; % Tracking error
    
end

function p_uncertain = add_uncertainties(p_nominal, uncertainty_percentage)
% =========================================================================
% ADD_UNCERTAINTIES
% Adds random uncertainty to the numerical parameters of a structure.
%
% INPUTS:
%   p_nominal             - The original structure containing nominal parameters.
%   uncertainty_percentage - The maximum percentage variation (e.g., 10 for +/-10%).
%
% OUTPUT:
%   p_uncertain           - A new structure with parameters randomly varied.
% =========================================================================
        % --- ADD THIS LINE ---
    rng(0, 'twister'); % Seed the generator for uncertainty (use any integer seed, e.g., 0)
    % ---------------------
    if uncertainty_percentage < 0
        error('Uncertainty percentage must be non-negative.');
    end

    p_uncertain = p_nominal; % Start by copying the nominal structure
    fields = fieldnames(p_nominal); % Get all parameter names

    fprintf('Applying +/-%g%% uncertainty to parameters...\n', uncertainty_percentage);

    for i = 1:length(fields)
        param_name = fields{i};
        original_value = p_nominal.(param_name);

        % Only apply uncertainty to scalar numerical values
        if isnumeric(original_value) && isscalar(original_value) && original_value ~= 0
            % Generate a random multiplier between (1 - perc/100) and (1 + perc/100)
            % rand() gives a number between 0 and 1.
            % 2*rand() - 1 gives a number between -1 and 1.
            random_factor = (uncertainty_percentage / 100) * (2 * rand() - 1);
            multiplier = 1 + random_factor;

            % Apply the multiplier
            p_uncertain.(param_name) = original_value * multiplier;

            % Optional: Display the change (can be commented out for cleaner output)
            % fprintf('  %s: %.4g -> %.4g (Multiplier: %.3f)\n', ...
            %         param_name, original_value, p_uncertain.(param_name), multiplier);
        end
    end
    fprintf('Uncertainty applied.\n');
end