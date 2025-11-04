
%% This script models, controls, and simulates a 3D ball-bot.
%
% Architecture:
%   - Path Planner: RRT* algorithm to find a collision-free path.
%   - Control System: A cascaded controller with a reference model.
%       - Outer Loop (P): A proportional controller for position tracking
%         that sets commanded velocities.
%       - Inner Loop (LQR + PI): An LQR controller generates a reference
%         torque and model, while a PI controller tracks the reference
%         velocity to reject disturbances.
%
% exportgraphics(gcf, 'my_sine_plot.pdf', 'ContentType', 'vector');
% =========================================================================
%% PART 0: SCRIPT INITIALIZATION
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

%% PART 2: INNER-LOOP LQR CONTROLLER DESIGN
% -------------------------------------------------------------------------
disp('Designing LQR controller...');
x_eq = zeros(10, 1);
u_eq = zeros(8, 1); %Motor Torques + External Wrench
[A_full, B_full] = linearizeBallbotModel(p, x_eq, u_eq);
% Decouple for Pitch/Roll Plane Controller
pitch_states_idx = [2, 5, 7, 10]; % [theta_y; phi_y; dot_theta_y; dot_phi_y]
pitch_input_idx = 2; % tau_y
A = A_full(pitch_states_idx, pitch_states_idx);
B = B_full(pitch_states_idx, pitch_input_idx);
% Define LQR cost matrices and compute gain
%diag([penalità_su_theta_y, penalità_su_phi_y, penalità_su_thetadot_y, penalità_su_phidot_y]
Q = diag([20 , 3, 26, 30]); 
R = 1;
[K, ~, ~] = lqr(A, B, Q, R);
disp('LQR Controller design complete.');
%% PART 3: PATH PLANNING
% Creation of a scalable 2D environment and path planning using RRT*.
% -------------------------------------------------------------------------
disp('Planning path through obstacle map...');
% --- Step 3.1: Define Map Scale and Base Geometry ---
map_scale = 0.5; % Change this value to resize the environment (e.g., 0.5 for half size, 2.0 for double)
base_map_size = [10, 10];               % Base size of the map in meters
base_obstacles_def = [2 1 2 4; 6 5 3 2]; % Base obstacle definitions [x y width height]
base_start_pos = [0, 0];
base_goal_pos = [8, 9];
% --- Step 3.2: Apply Scaling ---
map_size = base_map_size * map_scale;
obstacles_def = base_obstacles_def * map_scale;
start_pos = base_start_pos * map_scale;
goal_pos = base_goal_pos * map_scale;
% --- Step 3.3: Create the Map ---
map_resolution = 100; % cells/meter. Too low cause fail in planning
map = binaryOccupancyMap(map_size(1), map_size(2), map_resolution);
all_obstacle_points = [];
for k = 1:size(obstacles_def, 1)
    rect = obstacles_def(k, :);
    x_min = rect(1); y_min = rect(2);
    x_max = rect(1) + rect(3); y_max = rect(2) + rect(4);
    [X, Y] = meshgrid(x_min:1/map.Resolution:x_max, y_min:1/map.Resolution:y_max);
    obstacle_points = [X(:), Y(:)];
    all_obstacle_points = [all_obstacle_points; obstacle_points];
end
setOccupancy(map, all_obstacle_points, 1); %Fill obstacle cell's
%Inflation map around obstacles. sphere_radius + safety margin
%Needed becouse RRT plan trajectory for sphere's center
inflate(map, p.r_S + max(0.15 * map_scale , 0.3)); % Scale safety margin as well
% --- Step 3.4: Define State Space, Validator, and Planner ---
ss = stateSpaceSE2;
ss.StateBounds = [0 map_size(1); 0 map_size(2); -pi pi];
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1* map_scale;
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 0.5 * map_scale; %
planner.GoalReachedFcn = @(planner, q, q_target) norm(q(1:2) - q_target(1:2)) < 0.1 * map_scale;
planner.ContinueAfterGoalReached=true;
planner.GoalBias = 0.01;   % only 1% of the time sample the goal
planner.MaxIterations = 100000; %(default: 10000)
% --- Step 3.5: Plan the Path ---
start_state = [start_pos(1) start_pos(2) 0];
goal_state = [goal_pos(1) goal_pos(2) 0];
rng(100, 'twister'); % for repeatable results
[pathObj, solnInfo] = plan(planner, start_state, goal_state);
if ~solnInfo.IsPathFound
    error('No path found. Try adjusting planner parameters or map layout.');
end
waypoints = pathObj.States(:, 1:2);
goal_pos = waypoints(end, :)'; 
% --- Visualize the Planned Path ---
figure('Name', 'Planned Path with RRT* Exploration');
show(map);
hold on;
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', 'Color',[.7 .7 .7]);
plot(waypoints(:,1), waypoints(:,2), 'g-', 'LineWidth', 2);
plot(start_pos(1), start_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
plot(goal_pos(1), goal_pos(2), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
title('Planned Path with RRT* Exploration');
legend('Explored Tree', 'Final Path', 'Start', 'Goal', 'Location', 'best');
hold off;
%% PART 4: TRAJECTORY FOLLOWING SIMULATION (All-Simulink Version)
% -------------------------------------------------------------------------
disp('Running all-Simulink trajectory following simulation...');

% --- Setup Simulation Parameters in MATLAB Workspace ---
x0 = zeros(14, 1); % Initial state vector
x0(5) = start_pos(1) / p.r_S; % Initial phi_y for x-position
x0(4) = -start_pos(2) / p.r_S; % Initial phi_x for y-position

% Define controller parameters struct
%quanto aggressivamente il robot cerca di seguire i waypoints:
control_params.Kp = 0.3 ;
control_params.Ki= 0.05;
control_params.Kd= 0;

control_params.Kp_inner = 1;
control_params.Ki_inner =  8;

control_params.k_op = 0.2;

control_params.K = K;
control_params.A_lqr = A;
control_params.B_lqr = B;

% Define simulation time parameters
t_end = 160; % Max simulation time (seconds)
dt = 0.0005;  % Time step for the simulation (seconds)

% Define the name of your Simulink model file


%% Manipulator SetUp
modelName = 'scheme_pos_control_yawcontrol';
modelName = 'Copy_of_scheme_pos_control_yawcontrol';
manipulator_sim;

% --- NEW SECTION: GENERATE OPERATIONAL SPACE POSE TRAJECTORY ---
disp('Converting entire joint-space trajectory to operational space poses...');

% 1. Define the pose of the VIRTUAL, fixed base (Unchanged)
p_virtual_base = [goal_pos(1); goal_pos(2); p.h_U];
o_virtual_base = [0; 0; 0];
T_world_virtual_base = eul2tform(o_virtual_base', 'ZYX');
T_world_virtual_base(1:3, 4) = p_virtual_base;

% 2. Get the joint-space trajectory data (Unchanged)
joint_space_traj = manipulator_position_data;
num_points = size(joint_space_traj, 1);

% MODIFIED: Pre-allocate a 3D matrix to store the 4x4 pose at each time step
op_space_pose_trajectory = zeros(4, 4, num_points);

% 3. Loop through every point in the joint-space trajectory
for i = 1:num_points
    q_full_row = joint_space_traj(i, :);
    q_manip = [q_full_row(1:3), q_full_row(5:7)]';
    
    T_virtual_base_ee = getManipulatorFK(q_manip);
    
    % MODIFIED: Store the ENTIRE 4x4 matrix
    op_space_pose_trajectory(:,:,i) = T_world_virtual_base * T_virtual_base_ee;
end
%T_ref_timeseries = timeseries(op_space_pose_trajectory, manipulator_time_vector);
fprintf('Operational space reference pose trajectory (4x4xN) has been calculated.\n');
% --- END OF NEW SECTION ---
% --- Run the Simulink Simulation ---
% Configure the manipulator before:
simOut = sim(modelName, 'StopTime', num2str(t_end), 'FixedStep', num2str(dt));

disp('Simulation complete.');

%% PART 5: PLOTTING 
% -------------------------------------------------------------------------
% The simulation results are now in the 'simOut' object.
% We can access the logged data using the names of the 'To Workspace' blocks.

t_full = simOut.tout;

% For Timeseries format, the data is in the .Data property.
% We need to use permute() to fix the dimensions for plotting.
x_full = permute(simOut.x_sim.Data, [1, 3, 2]);
x_full = squeeze(x_full);
x_full = squeeze(x_full');

% --- Calculate Omni-Wheel Torques (Corrected for 2D Data) ---
disp('Calculating omni-wheel torque history...');

% The data from Simulink is now a 2D matrix where rows are time steps.
% No permute is needed.
tau_history = simOut.tau_sim.Data;

% Get the number of time steps from the number of ROWS.
num_time_steps = size(tau_history, 1); 

% Pre-allocate the output matrix.
tau_history_ow_cols = zeros(3, num_time_steps); 

% Loop through each TIME STEP (i.e., each row of tau_history).
for i = 1:num_time_steps
    
    % Get the i-th ROW and transpose it (') to create a 3x1 column vector.
    current_tau_S_col = tau_history(i, :)'; 
    
    % Call your conversion function, which expects a 3x1 vector.
    current_tau_W_col = T_3D_inv(p.r_S, p.r_ow, current_tau_S_col);
    
    % Store the 3x1 result in the i-th column of the output matrix.
    tau_history_ow_cols(:, i) = current_tau_W_col;
end

% --- FINAL TRANSPOSITION FOR PLOTTING ---
% Your myplot function expects data in an N x 3 format (time in rows).
% We must transpose our result matrices before passing them to the function.
tau_history_ow_for_plot = tau_history_ow_cols';

myplot(t_full, x_full', p, tau_history,tau_history_ow_for_plot);

% Define the obstacle height before calling the animation function.
obstacle_height = 0.5; % meters


%--------External wrench data plot ---------
ex_wrench = simOut.ex_wrench;
time_vector = ex_wrench.Time;
wrench_data = ex_wrench.Data;

% Extract each component into its own variable for clarity and easier plotting.
% The order must match how you constructed the vector in Simulink (e.g., with a Mux block).
F_px = squeeze(wrench_data(1, 1, :)); % Signal 1
F_py = squeeze(wrench_data(2, 1, :)); % Signal 2
F_pz = squeeze(wrench_data(3, 1, :)); % Signal 3
tau_px = squeeze(wrench_data(4, 1, :)); % Signal 4
tau_py = squeeze(wrench_data(5, 1, :)); % Signal 5

% --- 2. Create the Figure and Subplots ---
% Create a new figure window with a specific name.
figure('Name', 'External Wrench Analysis', 'NumberTitle', 'off');

% --- Subplot 1: External Forces ---
% Create a 2-row, 1-column grid of plots and select the first one (top).
subplot(2, 1, 1); 

plot(time_vector, F_px, 'LineWidth', 1.5, 'DisplayName', 'F_p_x (Forward)');
hold on; % Allows multiple lines to be plotted on the same axes
plot(time_vector, F_py, 'LineWidth', 1.5, 'DisplayName', 'F_p_y (Lateral)');
plot(time_vector, F_pz, 'LineWidth', 1.5, 'DisplayName', 'F_p_z (Vertical)');
hold off; % Prevents further plots on this axes

% Add details to make the plot easy to read
title('External Forces Applied by Manipulator');
xlabel('Time (s)');
ylabel('Force (N)');
legend('show', 'Location', 'best'); % Display the legend in the best location
grid on;
axis tight; % Fit the axes tightly around the data

% --- Subplot 2: External Torques ---
% Select the second plot area (bottom).
subplot(2, 1, 2);

plot(time_vector, tau_px, 'LineWidth', 1.5, 'DisplayName', '\tau_p_x (Roll Torque)');
hold on;
plot(time_vector, tau_py, 'LineWidth', 1.5, 'DisplayName', '\tau_p_y (Pitch Torque)');
hold off;

% Add details
title('External Torques Applied by Manipulator');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('show', 'Location', 'best');
grid on;
axis tight;
if(using_robust)
    %--------Manipualtion Control data plot ---------
    e_manip = simOut.e_manip;
    e_time_vector = e_manip.Time;
    e_manip_data = e_manip.Data;
    
    % Extract each component into its own variable for clarity and easier plotting.
    % The order must match how you constructed the vector in Simulink (e.g., with a Mux block).
    J_1 = squeeze(e_manip_data(1, 1, :)); % Signal 1
    J_2 = squeeze(e_manip_data(2, 1, :)); % Signal 2
    J_3 = squeeze(e_manip_data(3, 1, :)); % Signal 3
    J_4 = squeeze(e_manip_data(4, 1, :)); % Signal 4
    J_5 = squeeze(e_manip_data(5, 1, :)); % Signal 5
    J_6 = squeeze(e_manip_data(5, 1, :)); % Signal 6
    
    
    tau_manip = simOut.tau_manip;
    tau_time_vector = tau_manip.Time;
    e_manip_data = tau_manip.Data;
    
    % Extract each component into its own variable for clarity and easier plotting.
    % The order must match how you constructed the vector in Simulink (e.g., with a Mux block).
    tau_1 = squeeze(e_manip_data(1, 1, :)); % Signal 1
    tau_2 = squeeze(e_manip_data(2, 1, :)); % Signal 2
    tau_3 = squeeze(e_manip_data(3, 1, :)); % Signal 3
    tau_4 = squeeze(e_manip_data(4, 1, :)); % Signal 4
    tau_5 = squeeze(e_manip_data(5, 1, :)); % Signal 5
    tau_6 = squeeze(e_manip_data(5, 1, :)); % Signal 6
    
    figure('Name', 'Manipulator Control Analysis', 'NumberTitle', 'off');
    % --- Subplot 1:  Joint Errors ---
    % Create a 2-row, 1-column grid of plots and select the first one (top).
    subplot(2, 1, 1); hold on;
    
    plot(e_time_vector, J_1, 'LineWidth', 1.5, 'DisplayName', 'Shoulder Pan');
    plot(e_time_vector, J_2, 'LineWidth', 1.5, 'DisplayName', 'Shoulder Lift');
    plot(e_time_vector, J_3, 'LineWidth', 1.5, 'DisplayName', 'Elbow');
    plot(e_time_vector, J_4, 'LineWidth', 1.5, 'DisplayName', 'Wrist 1');
    plot(e_time_vector, J_5, 'LineWidth', 1.5, 'DisplayName', 'Wrist 2');
    plot(e_time_vector, J_6, 'LineWidth', 1.5, 'DisplayName', 'Wrist 3');
    hold off; % Prevents further plots on this axes
    
    % Add details to make the plot easy to read
    title('Joint Error on Manipulator');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('show', 'Location', 'best'); % Display the legend in the best location
    grid on;
    axis tight; % Fit the axes tightly around the data
    
    % --- Subplot 2: Control Torques ---
    % Select the second plot area (bottom).
    subplot(2, 1, 2);
    hold on;
    plot(tau_time_vector, tau_1, 'LineWidth', 1.5, 'DisplayName', '\tau_1 (Shoulder Pan Torque)');
    plot(tau_time_vector, tau_2, 'LineWidth', 1.5, 'DisplayName', '\tau_2 (Shoulder Lift Torque)');
    plot(tau_time_vector, tau_3, 'LineWidth', 1.5, 'DisplayName', '\tau_3 (Elbow Torque)');
    plot(tau_time_vector, tau_4, 'LineWidth', 1.5, 'DisplayName', '\tau_4 (Wrist 1 Torque)');
    plot(tau_time_vector, tau_5, 'LineWidth', 1.5, 'DisplayName', '\tau_5 (Wrist 2 Torque)');
    plot(tau_time_vector, tau_6, 'LineWidth', 1.5, 'DisplayName', '\tau_6 (Wrist 3 Torque)');
    hold off;
    
    % Add details
    title('Control Torques Applied on Manipulator');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    legend('show', 'Location', 'best');
    grid on;
    axis tight;
end

% --- START: OPERATIONAL SPACE ERROR PLOT ---
% from the 'ErrorPos' signal logged from Simulink.

    op_error = simOut.ErrorPos;
    error_time = op_error.Time;
    error_data = op_error.Data; % This is likely [3 x 1 x N]

    % 2. Extract each component using squeeze(), just like in your other plots
    err_x = squeeze(error_data(1, 1, :)); % 1st component (e.g., X error)
    err_y = squeeze(error_data(2, 1, :)); % 2nd component (e.g., Y error)
    err_z = squeeze(error_data(3, 1, :)); % 3rd component (e.g., Z error)

    % 3. Create the new figure and plot the data
    figure('Name', 'Manipulator Operational Space Error', 'NumberTitle', 'off');
    
    plot(error_time, err_x, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error X');
    hold on;
    plot(error_time, err_y, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Error Y');
    plot(error_time, err_z, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error Z');
    hold off;

    % 4. Add labels, title, and legend
    title('Operational Space Position Error (e_p)');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('show', 'Location', 'best');
    grid on;
    axis tight;
    
% --- END: OPERATIONAL SPACE ERROR PLOT ---
% --- START: NEW 6-PLOT SUMMARY FIGURES (SPLIT IN TIME) ---
disp('Generating split-time summary plots with zoomed axes...');

% 1. Define split time and zoom limits
t_split = 60.0;
% --- ADJUST YOUR ZOOM-LEVELS HERE ---
% Sets the Y-axis for torques to [-limit, +limit]
zoom_torque_limit = 5.0; 
% Sets the Y-axis for forces to [-limit, +limit]
zoom_force_limit = 8.0;  
% --------------------------------------

% 2. Find logical indices for different time vectors
% Indices for 'x_full' and 'tau_history_ow_for_plot' (which use t_full)
idx1_main = (t_full <= t_split);
idx2_main = (t_full > t_split);

% Indices for 'ex_wrench' data (which uses time_vector)
idx1_wrench = (time_vector <= t_split);
idx2_wrench = (time_vector > t_split);

% --- FIGURE 1: NAVIGATION PHASE (0-60s) ---
figure('Name', 'Robot State (Navigation: 0-60s)', 'NumberTitle', 'off', 'WindowState', 'maximized');
sgtitle('Robot State Summary (Navigation Phase: 0s - 60s)');

% (1,1) Tilt Angles
subplot(3, 2, 1);
plot(t_full(idx1_main), rad2deg(x_full(1, idx1_main)), 'r-', 'LineWidth', 1.5); % theta_x
hold on;
plot(t_full(idx1_main), rad2deg(x_full(2, idx1_main)), 'b-', 'LineWidth', 1.5); % theta_y
hold off; grid on; axis tight;
title('Upper Body Tilt Angles');
xlabel('Time (s)'); ylabel('Angle (degrees)');
legend('\theta_x (Roll)', '\theta_y (Pitch)', 'Interpreter', 'tex', 'Location', 'best');

% (2,1) Tilt Velocity
subplot(3, 2, 3);
plot(t_full(idx1_main), x_full(6, idx1_main), 'r-', 'LineWidth', 1.5); % dot_theta_x
hold on;
plot(t_full(idx1_main), x_full(7, idx1_main), 'b-', 'LineWidth', 1.5); % dot_theta_y
hold off; grid on; axis tight;
title('Upper Body Tilt Velocity');
xlabel('Time (s)'); ylabel('Speed (rad/s)');
legend('d\theta_x/dt', 'd\theta_y/dt', 'Interpreter', 'tex', 'Location', 'best');

% (3,1) Yaw Angle
subplot(3, 2, 5);
plot(t_full(idx1_main), rad2deg(x_full(3, idx1_main)), 'm-', 'LineWidth', 1.5); % theta_z (magenta)
grid on; axis tight;
title('Yaw Angle');
xlabel('Time (s)'); ylabel('Angle (degrees)');
legend('\theta_z (Yaw)', 'Interpreter', 'tex', 'Location', 'best');

% (1,2) SW Angular Speed
subplot(3, 2, 2);
plot(t_full(idx1_main), x_full(9, idx1_main), 'r-', 'LineWidth', 1.5); % dot_phi_x
hold on;
plot(t_full(idx1_main), x_full(10, idx1_main), 'b-', 'LineWidth', 1.5); % dot_phi_y
hold off; grid on; axis tight;
title('Spherical Wheel Angular Speeds');
xlabel('Time (s)'); ylabel('Speed (rad/s)');
legend('d\phi_x/dt', 'd\phi_y/dt', 'Interpreter', 'tex', 'Location', 'best');

% (2,2) OW Control Torques (ZOOMED)
subplot(3, 2, 4);
% Find true peaks before plotting
tau1 = tau_history_ow_for_plot(idx1_main, 1);
tau2 = tau_history_ow_for_plot(idx1_main, 2);
tau3 = tau_history_ow_for_plot(idx1_main, 3);
all_taus = [tau1; tau2; tau3];
max_peak_t = max(all_taus, [], 'all', 'omitnan');
min_peak_t = min(all_taus, [], 'all', 'omitnan');
% Plot the data
plot(t_full(idx1_main), tau1, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_full(idx1_main), tau2, 'g-', 'LineWidth', 1.5);
plot(t_full(idx1_main), tau3, 'b-', 'LineWidth', 1.5);
hold off; grid on;
% Apply zoom and report peaks in title
ylim([-zoom_torque_limit, zoom_torque_limit]);
title(sprintf('Applied Torques (Zoomed)\nActual Peaks: [%.1f, %.1f] Nm', min_peak_t, max_peak_t));
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('\tau_1', '\tau_2', '\tau_3', 'Interpreter', 'tex', 'Location', 'best');

% (3,2) External Forces (ZOOMED)
subplot(3, 2, 6);
% Find true peaks before plotting
fpx = F_px(idx1_wrench);
fpy = F_py(idx1_wrench);
fpz = F_pz(idx1_wrench);
all_forces = [fpx; fpy; fpz];
max_peak_f = max(all_forces, [], 'all', 'omitnan');
min_peak_f = min(all_forces, [], 'all', 'omitnan');
% Plot the data
plot(time_vector(idx1_wrench), fpx, 'LineWidth', 1.5, 'DisplayName', 'F_p_x (Forward)');
hold on;
plot(time_vector(idx1_wrench), fpy, 'LineWidth', 1.5, 'DisplayName', 'F_p_y (Lateral)');
plot(time_vector(idx1_wrench), fpz, 'LineWidth', 1.5, 'DisplayName', 'F_p_z (Vertical)');
hold off; grid on;
% Apply zoom and report peaks in title
ylim([-zoom_force_limit, zoom_force_limit]);
title(sprintf('External Forces (Zoomed)\nActual Peaks: [%.1f, %.1f] N', min_peak_f, max_peak_f));
xlabel('Time (s)'); ylabel('Force (N)');
legend('show', 'Location', 'best');

% --- FIGURE 2: MANIPULATION PHASE (60s-End) ---
if any(idx2_main) % Only plot if there is data after 60s
    figure('Name', 'Robot State (Manipulation: 60s-End)', 'NumberTitle', 'off', 'WindowState', 'maximized');
    sgtitle('Robot State Summary (Manipulation Phase: 60s - End)');
    
    % (1,1) Tilt Angles
    subplot(3, 2, 1);
    plot(t_full(idx2_main), rad2deg(x_full(1, idx2_main)), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_full(idx2_main), rad2deg(x_full(2, idx2_main)), 'b-', 'LineWidth', 1.5);
    hold off; grid on; axis tight;
    title('Upper Body Tilt Angles');
    xlabel('Time (s)'); ylabel('Angle (degrees)');
    legend('\theta_x (Roll)', '\theta_y (Pitch)', 'Interpreter', 'tex', 'Location', 'best');
    
    % (2,1) Tilt Velocity
    subplot(3, 2, 3);
    plot(t_full(idx2_main), x_full(6, idx2_main), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_full(idx2_main), x_full(7, idx2_main), 'b-', 'LineWidth', 1.5);
    hold off; grid on; axis tight;
    title('Upper Body Tilt Velocity');
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    legend('d\theta_x/dt', 'd\theta_y/dt', 'Interpreter', 'tex', 'Location', 'best');
    
    % (3,1) External Torques (FIXED Y-AXIS) <-- REPLACED Yaw Angle
    subplot(3, 2, 5);
    if any(idx2_wrench)
        % Find true peaks before plotting
        tpx_p2 = tau_px(idx2_wrench);
        tpy_p2 = tau_py(idx2_wrench);
        all_ext_torques_p2 = [tpx_p2; tpy_p2];
        max_peak_et2 = max(all_ext_torques_p2, [], 'all', 'omitnan');
        min_peak_et2 = min(all_ext_torques_p2, [], 'all', 'omitnan');
        
        % Plot the data
        plot(time_vector(idx2_wrench), tpx_p2, 'LineWidth', 1.5, 'DisplayName', '\tau_p_x (Roll Torque)');
        hold on;
        plot(time_vector(idx2_wrench), tpy_p2, 'LineWidth', 1.5, 'DisplayName', '\tau_p_y (Pitch Torque)');
        hold off;
        legend('show', 'Location', 'best');
        
        % Apply fixed Y-axis window as requested
        ylim([-20, 40]);
        % Report the true peaks in the title, which may be outside the window
        title(sprintf('External Torques (Window: [-20, 40])\nActual Peaks: [%.1f, %.1f] Nm', min_peak_et2, max_peak_et2));
    else
        title('External Torques (No data for t > 60s)');
    end
    grid on;
    xlabel('Time (s)'); ylabel('Torque (Nm)');
    
    % (1,2) SW Angular Speed
    subplot(3, 2, 2);
    plot(t_full(idx2_main), x_full(9, idx2_main), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_full(idx2_main), x_full(10, idx2_main), 'b-', 'LineWidth', 1.5);
    hold off; grid on; axis tight;
    title('Spherical Wheel Angular Speeds');
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    legend('d\phi_x/dt', 'd\phi_y/dt', 'Interpreter', 'tex', 'Location', 'best');
    
    % (2,2) OW Control Torques (ZOOMED)
    subplot(3, 2, 4);
    % Find true peaks before plotting
    tau1_p2 = tau_history_ow_for_plot(idx2_main, 1);
    tau2_p2 = tau_history_ow_for_plot(idx2_main, 2);
    tau3_p2 = tau_history_ow_for_plot(idx2_main, 3);
    all_taus_p2 = [tau1_p2; tau2_p2; tau3_p2];
    max_peak_t2 = max(all_taus_p2, [], 'all', 'omitnan');
    min_peak_t2 = min(all_taus_p2, [], 'all', 'omitnan');
    % Plot the data
    plot(t_full(idx2_main), tau1_p2, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_full(idx2_main), tau2_p2, 'g-', 'LineWidth', 1.5);
    plot(t_full(idx2_main), tau3_p2, 'b-', 'LineWidth', 1.5);
    hold off; grid on;
    % Apply zoom and report peaks in title
    ylim([-zoom_torque_limit, zoom_torque_limit]);
    title(sprintf('Applied Torques (Zoomed)\nActual Peaks: [%.1f, %.1f] Nm', min_peak_t2, max_peak_t2));
    xlabel('Time (s)'); ylabel('Torque (Nm)');
    legend('\tau_1', '\tau_2', '\tau_3', 'Interpreter', 'tex', 'Location', 'best');
    
    % (3,2) External Forces (ZOOMED)
    subplot(3, 2, 6);
    if any(idx2_wrench)
        % Find true peaks before plotting
        fpx_p2 = F_px(idx2_wrench);
        fpy_p2 = F_py(idx2_wrench);
        fpz_p2 = F_pz(idx2_wrench);
        all_forces_p2 = [fpx_p2; fpy_p2; fpz_p2];
        max_peak_f2 = max(all_forces_p2, [], 'all', 'omitnan');
        min_peak_f2 = min(all_forces_p2, [], 'all', 'omitnan');
        % Plot the data
        plot(time_vector(idx2_wrench), fpx_p2, 'LineWidth', 1.5, 'DisplayName', 'F_p_x (Forward)');
        hold on;
        plot(time_vector(idx2_wrench), fpy_p2, 'LineWidth', 1.5, 'DisplayName', 'F_p_y (Lateral)');
        plot(time_vector(idx2_wrench), fpz_p2, 'LineWidth', 1.5, 'DisplayName', 'F_p_z (Vertical)');
        hold off;
        legend('show', 'Location', 'best');
        % Apply zoom and report peaks in title
        ylim([-zoom_force_limit, zoom_force_limit]);
        title(sprintf('External Forces (Zoomed)\nActual Peaks: [%.1f, %.1f] N', min_peak_f2, max_peak_f2));
    else
        title('External Forces (No data for t > 60s)');
    end
    grid on;
    xlabel('Time (s)'); ylabel('Force (N)');
else
    fprintf('No data for Phase 2 (t > 60s). Skipping manipulation phase plot.\n');
end
% --- END: NEW 6-PLOT SUMMARY FIGURES ---
% --- START: NEW ERROR SUMMARY PLOT (NAVIGATION vs MANIPULATION) ---
disp('Generating split-time error summary plot...');

% --- 1. Extract Navigation Error (from new logged signal) ---
nav_error = simOut.nav_error_sim;
nav_error_time = nav_error.Time;
% Filter for navigation phase (t <= 60s)
idx_nav_error = (nav_error_time <= t_split);
time_nav = nav_error_time(idx_nav_error);
% Extract X and Y components
err_nav_x = nav_error.Data(1, 1, idx_nav_error);
err_nav_y = nav_error.Data(2, 1, idx_nav_error);
% Squeeze to remove extra dimensions
err_nav_x = squeeze(err_nav_x);
err_nav_y = squeeze(err_nav_y);

% --- 2. Extract Manipulation Error (from existing signal) ---
idx_op_error_2 = (error_time > t_split); 
time_manip = error_time(idx_op_error_2);
% Filter the error data for the manipulation phase
err_x_manip = err_x(idx_op_error_2);
err_y_manip = err_y(idx_op_error_2);
err_z_manip = err_z(idx_op_error_2);

% --- 3. Create the new figure ---
figure('Name', 'Error Profile (Navigation vs. Manipulation)', 'NumberTitle', 'off');

% --- Plot 1 (Left): Waypoint Tracking Error (0-60s) ---
subplot(1, 2, 1);
plot(time_nav, err_nav_x, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error X (Sagittal)');
hold on;
plot(time_nav, err_nav_y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error Y (Frontal)');
hold off;
grid on;
axis tight;
title(sprintf('Waypoint Tracking Error (0 - %.0fs)', t_split));
xlabel('Time (s)');
ylabel('Error (m)');
legend('show', 'Location', 'best');

% --- Plot 2 (Right): Operational Space Error (>60s) ---
subplot(1, 2, 2);
plot(time_manip, err_x_manip, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error X');
hold on;
plot(time_manip, err_y_manip, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Error Y');
plot(time_manip, err_z_manip, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error Z');
hold off;
grid on;
axis tight;
title(sprintf('Operational Space Error (t > %.0fs)', t_split));
xlabel('Time (s)');
ylabel('Error (m)');
legend('show', 'Location', 'best');

% ---
% -------- START: TRACKING PERFORMANCE METRICS CALCULATION --------
disp('Calculating tracking performance metrics...');
% --- 1. Extract the robot's actual trajectory ---
pos_x_actual = p.r_S * x_full(5, :);
pos_y_actual = -p.r_S * x_full(4, :);
actual_path = [pos_x_actual', pos_y_actual'];
% --- 2. Get the reference trajectory ---
ref_path = waypoints;
% --- 3. Calculate the error for each point of the actual trajectory ---
num_actual_points = size(actual_path, 1);
squared_error_dist = zeros(num_actual_points, 1);
for i = 1:num_actual_points
    distances_to_ref_sq = sum((ref_path - actual_path(i, :)).^2, 2);
    squared_error_dist(i) = min(distances_to_ref_sq);
end
% --- 4. Calculate the final metrics ---
rmse_tracking = sqrt(mean(squared_error_dist));
mae_tracking = sqrt(max(squared_error_dist));
% --- 5. Display the results in the console ---
fprintf('\n----------------------------------------\n');
fprintf('  Tracking Performance Metrics\n');
fprintf('----------------------------------------\n');
fprintf('Root Mean Square Error (RMSE): %.4f meters\n', rmse_tracking);
fprintf('Maximum Absolute Error (MAE):  %.4f meters\n', mae_tracking);
fprintf('----------------------------------------\n\n');

% -------- NUOVA AGGIUNTA: IDENTIFICAZIONE DELLA FASE DI NAVIGAZIONE --------
disp('Identifying navigation phase end time...');
% Definisci il penultimo waypoint e la soglia per considerarlo "raggiunto"
penultimate_wp = waypoints(end-1, :);
reach_threshold = 0.5 * map_scale; % Usa una soglia simile a quella del planner

% Trova il primo indice temporale in cui il robot si avvicina al penultimo waypoint
navigation_end_idx = find(sqrt(sum((actual_path - penultimate_wp).^2, 2)) < reach_threshold, 1, 'first');

% Gestisci il caso in cui il waypoint non venga mai raggiunto
if isempty(navigation_end_idx)
    warning('Penultimate waypoint not reached. Calculating mean tilt over full duration.');
    navigation_end_idx = length(t_full); % Se non lo raggiunge, usa l'intera durata
end
fprintf('Navigation phase ends at t = %.2f seconds.\n', t_full(navigation_end_idx));


% -------- SEZIONE MODIFICATA: TILT ANGLE PERFORMANCE METRICS CALCULATION --------
disp('Calculating tilt angle performance metrics...');
% --- 1. Estrai la cronologia completa degli angoli di inclinazione ---
theta_x_rad = x_full(1, :);
theta_y_rad = x_full(2, :);
theta_x_deg = rad2deg(theta_x_rad);
theta_y_deg = rad2deg(theta_y_rad);

% --- 2. Filtra i dati per la sola fase di navigazione ---
nav_indices = 1:navigation_end_idx;
nav_theta_x_deg = theta_x_deg(nav_indices);
nav_theta_y_deg = theta_y_deg(nav_indices);

% --- 3. Calcola le Metriche ---
% Il TILT MEDIO ora viene calcolato SOLO sui dati della fase di navigazione
mean_abs_nav_theta_x = mean(abs(nav_theta_x_deg));
mean_abs_nav_theta_y = mean(abs(nav_theta_y_deg));

% Il TILT MASSIMO viene ancora calcolato sull'INTERA traiettoria come metrica globale
max_abs_theta_x  = max(abs(theta_x_deg));
max_abs_theta_y  = max(abs(theta_y_deg));

% --- 4. Mostra i risultati nella console (con le nuove etichette) ---
fprintf('\n----------------------------------------\n');
fprintf('  Tilt Angle Performance Metrics\n');
fprintf('----------------------------------------\n');
fprintf('Roll Angle (Theta_x / Transversal Plane):\n');
fprintf('  - Mean Navigation Tilt: %.4f degrees\n', mean_abs_nav_theta_x);
fprintf('  - Max Absolute Tilt:    %.4f degrees\n', max_abs_theta_x);
fprintf('\n');
fprintf('Pitch Angle (Theta_y / Sagittal Plane):\n');
fprintf('  - Mean Navigation Tilt: %.4f degrees\n', mean_abs_nav_theta_y);
fprintf('  - Max Absolute Tilt:    %.4f degrees\n', max_abs_theta_y);
fprintf('----------------------------------------\n\n');

%% AND ANIMATION
wrench_data_for_animation = squeeze(wrench_data)';
q_full = permute(simOut.q.Data,[1 3 2]);

% animateBallbot_Manipulator(t_full, x_full', q_full, p, map_size, map_scale, waypoints, obstacles_def)
