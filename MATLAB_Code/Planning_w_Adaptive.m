%% SCRIPT TO SIMULATE PATH PLANNING AND TRAJECTORY FOLLOWING
% =========================================================================
% This script models, controls, and simulates a 3D ball-bot.
%
% Architecture:
%   - Path Planner: RRT* algorithm to find a collision-free path.
%   - %% --- MODIFIED --- %%
%   - Control System:
% This script uses our full suite of modular and auto-generated functions.
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

% Calculate the distance between the center of the sphere and the force's application point
p.h_app = p.h_U - p.r_S; % Distance from the center of the sphere to the force application point
%% PART 2: BUILD THE LINEARIZED MODEL
% --- Step 1: Linearize the Dynamic Model ---
x_eq = zeros(10, 1);
u_eq = zeros(3, 1);
[A, B] = linearizeBallbotModel_MRAC(p, x_eq, u_eq);


% Decouple for Roll Plane Controller
roll_states_idx = [1, 2, 3, 4]; % [theta_x; dot_theta_x; phi_x; dot_phi_x]
roll_input_idx = 1; % tau_x 
A_roll = A(roll_states_idx, roll_states_idx);
B_roll = B(roll_states_idx, roll_input_idx);

% Decouple for Pitch Plane Controller
pitch_states_idx = [5, 6, 7, 8]; % [theta_y; dot_theta_y; phi_y; dot_phi_y]
pitch_input_idx = 2; % tau_y
A_pitch = A(pitch_states_idx, pitch_states_idx);
B_pitch = B(pitch_states_idx,pitch_input_idx);

% Decouple for Spin Plane Controller
spin_states_idx = [9,10]; % [theta_z; dot_theta_z;]
spin_input_idx = 3; % tau_z
A_spin = A(spin_states_idx, spin_states_idx);
B_spin = B(spin_states_idx, spin_input_idx);
%% PART 3: REFERENCE MODEL DESIGN

%Retrieve Reference Model for Saggital and Frontal Plane Dynamics
[Am_y, Bm_y, Cm_y, Dm_y] = create_reference_model(A_pitch,B_pitch);


disp('Reference Model for Saggital Plane design complete.');
disp('Am_y = ');
disp(Am_y);
disp('Bm_y = ');
disp(Bm_y);

[Am_x, Bm_x, Cm_x, Dm_x] = create_reference_model(A_roll,B_roll);


disp('Reference Model for Frontal Plane design complete.');
disp('Am_x = ');
disp(Am_x);
disp('Bm_x = ');
disp(Bm_x);


[Am_z, Bm_z, Cm_z, Dm_z] = create_reference_model(A_spin,B_spin);


disp('Reference Model for Transverse Plane design complete.');
disp('Am_z = ');
disp(Am_z);
disp('Bm_z = ');
disp(Bm_z);
%% PART 3: PATH PLANNING
% Creation of a scalable 2D environment and path planning using RRT*.
% -------------------------------------------------------------------------
disp('Planning path through obstacle map...');
% --- Step 3.1: Define Map Scale and Base Geometry ---
map_scale = 0.5; % Change this value to resize the environment (e.g., 0.5 for half size, 2.0 for double)
base_map_size = [10, 10];               % Base size of the map in meters
base_obstacles_def = [2 1 2 4; 6 5 3 2]; % Base obstacle definitions [x y width height]
base_start_pos = [1, 1];
base_goal_pos = [8, 9];
% --- Step 3.2: Apply Scaling ---
map_size = base_map_size * map_scale;
obstacles_def = base_obstacles_def * map_scale;
% start_pos = base_start_pos * map_scale;
start_pos = [0; 0];
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
inflate(map, p.r_S + 0.15 * map_scale); % Scale safety margin as well
% --- Step 3.4: Define State Space, Validator, and Planner ---
ss = stateSpaceSE2;
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
t = 0:1:size(waypoints,1)-1;
waypoints_ts = timeseries(waypoints, t);
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
x0 = zeros(10, 1); % Initial state vector
x0(1) = pi/24;
x0(5) = pi/30;
x0(9) = pi/6;
x0(7) = start_pos(1) / p.r_S; % Initial phi_y for x-position
x0(3) = -start_pos(2) / p.r_S; % Initial phi_x for y-position


% Define simulation time parameters
t_end = t(end)+10; % Max simulation time (seconds)
dt = 0.0001;  % Time step for the simulation (seconds)
% --- Adaptive Controller Parameters ---
alpha = 100;
beta = 2;
gamma = 10;

% Solve Lyapunov Equation for P (Saggital/Frontal)
Q = eye(4);
Px = lyap(Am_x', Q);
Py = lyap(Am_y', Q);

% Solve Lyapunov Equation for P (Transverse)
Qz = eye(2);
Pz = lyap(Am_z', Qz);

%% Manipulator SetUp
modelName = 'scheme_w_Adaptive';
manipulator_sim;
open_system(modelName);

% --- Run the Simulink Simulation ---
% Configure the manipulator before:
simOut = sim(modelName, 'StopTime', num2str(t_end), 'FixedStep', num2str(dt));

disp('Simulation complete.');


%% PART 5: PLOTTING AND ANIMATION
% -------------------------------------------------------------------------
% The simulation results are now in the 'simOut' object.
% We can access the logged data using the names of the 'To Workspace' blocks.

t_full = simOut.tout;

x_full = permute(simOut.x_sim.signals.values,[3 1 2]);
e_full = permute(simOut.e_sim.signals.values,[3 1 2]); 
tau_input  = permute(simOut.tau_sim.signals.values,[1 2 3]); 
r_full = permute(simOut.r_sim.signals.values,[1 2 3]); 
Kr_hat = permute(simOut.Kr_hat_sim.signals.values,[1 2 3]);
K_hat = permute(simOut.K_hat_sim.signals.values,[3 2 1]);
Ke_hat = permute(simOut.Ke_hat_sim.signals.values,[1 2 3]);


gains_hystory = [Kr_hat,K_hat,Ke_hat];
% --- Calculate Omni-Wheel Torques ---
disp('Calculating omni-wheel torque history...');

% Get the number of time steps from the columns
num_time_steps = size(tau_input, 1); 

% Pre-allocate the output matrix with the same orientation (3 x N)
tau_history_ow_cols = zeros(num_time_steps,3); 

% Loop through each TIME STEP (i.e., each column)
for i = 1:num_time_steps
    
    % Get the i-th column, which is already the correct 3x1 format
    current_tau_S_col = tau_input(i,:); 
    
    % Call your conversion function.
    current_tau_W_col = T_3D_inv(p.r_S, p.r_ow, current_tau_S_col');
    
    % Store the 3x1 result in the i-th column of the output matrix.
    tau_history_ow_cols(i,:) = current_tau_W_col';
end
disp('Calculation complete.');

% --- FINAL TRANSPOSITION FOR PLOTTING ---
% Your myplot function expects data in an N x 3 format (time in rows).
% We must transpose our result matrices before passing them to the function.

% tau_history_ow_for_plot = tau_history_ow_cols';

myplot_MRAC(t_full, x_full, r_full, p,tau_history_ow_cols, gains_hystory);


function [Am, Bm, Cm, Dm] = create_reference_model(A,B)
%  Creates a reference model for a ballbot's planar (4th-order) or 
%  yaw (2nd-order) motion.
%
%   This function detects the system order from the input matrices (A, B)
%   and uses pole placement to design a stable, well-behaved reference model.
%
%   INPUTS:
%   A       - Linearized State Matrix (2x2 for Yaw, 4x4 for Planar)
%   B       - Linearized Input Matrix (2x1 for Yaw, 4x1 for Planar)
%
%   OUTPUTS:
%   Am      - The reference model state matrix (NxN).
%   Bm      - The reference model input matrix (Nx1).
%   Cm      - The reference model output matrix (NxN Identity).
%   Dm      - The reference model feedthrough matrix (Nx1 Zero).
%
    
    % Get the order of the system
    system_order = size(A, 1); 
    
    if system_order == 2
        %% -----------------------------------------------------------------
        %  Case 1: 2nd-Order Model (Transverse/Yaw)
        %  -----------------------------------------------------------------
        disp('Designing 2nd-order (Yaw) Reference Model...');
        
        % --- Define Desired Behavior (TUNABLE PARAMETERS) ---
        Ts_z = 3; % Settling time for Yaw Angle Positioning
        omega_z = 4 / Ts_z;
        zeta_z = 0.8; % Critically damped
        
        % Calculate the two desired pole locations (repeated real pole)
        poles = roots([1, 2*zeta_z*omega_z, omega_z^2]);
        
        % Calculate the ideal feedback gain 'Km'
        Km = place(A, B, poles);
        
        % The reference model's state matrix is the stable closed-loop system
        Am = A - B*Km;
        
        % Calculate feedforward gain 'N' for unity steady-state gain
        % Assumes the output to track is the first state (yaw angle)
        C_output_yaw = [1, 0]; 
        N = -1 / (C_output_yaw * inv(Am) * B);
        
        % The reference model's input matrix
        Bm = B * N;
        
        % Output the full reference state
        Cm = eye(2);
        Dm = zeros(2,1);
        
        disp('Second-order reference model created successfully.');

    elseif system_order == 4
        %% -----------------------------------------------------------------
        %  Case 2: 4th-Order Model (Frontal/Sagittal Plane)
        %  -----------------------------------------------------------------
        disp('Designing 4th-order (Planar) Reference Model...');
        
        % --- Define Desired Behavior (TUNABLE PARAMETERS) ---
        Ts_position = 3;  % Desired settling time for the ball's position.
        Ts_balance_ratio = 3; % How many times faster balancing should be.
        Ts_balance = Ts_position / Ts_balance_ratio; % Settling time for tilt.
        
        omega_np = 4 / (Ts_position); % Natural frequency for position
        omega_nb = 4 / (Ts_balance); % Natural frequency for balance
        
        zeta_p = 0.97;      % Damping ratio for position
        zeta_b = 1.5;       % Damping ratio for balance
             
        % Calculate the four desired pole locations
        p_slow = roots([1, 2*zeta_p*omega_np, omega_np^2]);
        p_fast = roots([1, 2*zeta_b*omega_nb, omega_nb^2]);
        poles = [p_slow; p_fast]; % Vector of 4 desired pole locations
        
        % Calculate the ideal feedback gain 'Km'
        Km = place(A, B, poles);
        
        % The reference model's state matrix
        Am = A - B*Km;
        
        % Calculate feedforward gain 'N' for unity steady-state gain
        % Assumes output to track is the third state (phi, ball position/angle)
        C_output_phi = [0, 0, 1, 0]; 
        N = -1 / (C_output_phi * inv(Am) * B);
        
        % The reference model's input matrix
        Bm = B * N;
        
        % Output the full reference state
        Cm = eye(4);
        Dm = zeros(4,1);
        
        disp('Fourth-order reference model created successfully.');
    end
    
end
