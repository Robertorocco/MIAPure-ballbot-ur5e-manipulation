%% SCRIPT TO TUNE MRAC ADAPTATION PARAMETERS
% =========================================================================
% This script uses a GRID SEARCH to find the optimal 'alpha', 'beta',
% and 'gamma' parameters for the MRAC controller based on a weighted 
% cost function of user-defined metrics.
% =========================================================================

%% PART 0: SCRIPT INITIALIZATION
clear; clc; close all;
addpath('function'); 
modelName = 'scheme_w_Adaptive';

%% PART 1: SYSTEM PARAMETERS
ur5e_mass = 20.7;
robotiq_gripper_mass = 1;
p.h_U = 0.50; p.m_S = 3.6; p.I_S = 0.047; p.r_S = 0.1145;
p.m_C = 17.9 - p.m_S; p.COM_C = (p.h_U-2*p.r_S)/2+p.r_S; 
p.m_R = ur5e_mass + robotiq_gripper_mass; p.COM_R= 0.7649;
p.m_U = p.m_C + p.m_R; p.I_Ux = 0.1848 + 14.4706; p.I_Uy = 0.1848 + 15.0733;
p.Iz = 0.0085 + 1.2023;
p.l_U = (p.m_C * p.COM_C + p.m_R * p.COM_R) / p.m_U;
p.g = 9.81; p.b_theta = 0.0; p.b_phi = 1; p.fz = 1; p.r_ow=0.0625;
p.h_app = p.h_U - p.r_S; 

%% PART 2: BUILD THE LINEARIZED MODEL
x_eq = zeros(10, 1); u_eq = zeros(3, 1);
[A, B] = linearizeBallbotModel_MRAC(p, x_eq, u_eq);
roll_states_idx = [1, 2, 3, 4]; roll_input_idx = 1; 
A_roll = A(roll_states_idx, roll_states_idx);
B_roll = B(roll_states_idx, roll_input_idx);
pitch_states_idx = [5, 6, 7, 8]; pitch_input_idx = 2;
A_pitch = A(pitch_states_idx, pitch_states_idx);
B_pitch = B(pitch_states_idx,pitch_input_idx);
spin_states_idx = [9,10]; spin_input_idx = 3;
A_spin = A(spin_states_idx, spin_states_idx);
B_spin = B(spin_states_idx, spin_input_idx);

%% PART 3: REFERENCE MODEL DESIGN
[Am_y, Bm_y, Cm_y, Dm_y] = create_reference_model(A_pitch,B_pitch);
[Am_x, Bm_x, Cm_x, Dm_x] = create_reference_model(A_roll,B_roll);
[Am_z, Bm_z, Cm_z, Dm_z] = create_reference_model(A_spin,B_spin);
%% PART 4: PATH PLANNING
map_scale = 0.5; base_map_size = [10, 10];
base_obstacles_def = [2 1 2 4; 6 5 3 2]; 
base_goal_pos = [8, 9];
map_size = base_map_size * map_scale;
obstacles_def = base_obstacles_def * map_scale;
start_pos = [0; 0];
goal_pos = base_goal_pos * map_scale;
map_resolution = 100; 
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
setOccupancy(map, all_obstacle_points, 1);
inflate(map, p.r_S + 0.15 * map_scale); 
ss = stateSpaceSE2; sv = validatorOccupancyMap(ss);
sv.Map = map; sv.ValidationDistance = 0.1* map_scale;
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 0.5 * map_scale; 
planner.GoalReachedFcn = @(planner, q, q_target) norm(q(1:2) - q_target(1:2)) < 0.1 * map_scale;
planner.ContinueAfterGoalReached=true; planner.GoalBias = 0.01;
planner.MaxIterations = 100000;
start_state = [start_pos(1) start_pos(2) 0];
goal_state = [goal_pos(1) goal_pos(2) 0];
rng(100, 'twister');
[pathObj, solnInfo] = plan(planner, start_state, goal_state);
if ~solnInfo.IsPathFound
    error('No path found. Try adjusting planner parameters or map layout.');
end
waypoints = pathObj.States(:, 1:2);
t = 0:1:size(waypoints,1)-1;
waypoints_ts = timeseries(waypoints, t);

%% PART 5: OPTIMIZATION SETUP
% -------------------------------------------------------------------------
disp('Setting up optimization...');
x0 = zeros(10, 1);
x0(1) = pi/24;
x0(9) = pi/4;
x0(7) = start_pos(1) / p.r_S; x0(3) = -start_pos(2) / p.r_S; 
t_end = t(end)+6; dt = 0.01;  

Q = eye(4); P_x = lyap(Am_x', Q); P_y = lyap(Am_y', Q);
Qz = eye(2); Pz = lyap(Am_z', Qz);
% --- Load and Open Simulink Model ---
manipulator_sim;
load_system(modelName);
% --- Define Optimization Weights ---
weights.itae = 1.0;            % Tracking Error (ITAE)
weights.settling_time = 0.5;   % Settling time of adaptive gains
weights.overshoot = 0.5;       % Max tilt angle (stability)
weights.control_effort = 0.8;  % Energy usage (L2-norm of tau)
weights.saturation_penalty = 1.0; % Max actuator torque (saturation penalty)

% --- Define Initial Parameter Guess ---
initial_guess = [50; 5; 10]; % [alpha; beta; gamma]
% --- Get Baseline Metrics ---
disp('Running baseline simulation to establish metric scales...');
[~, baseline_metrics] = calculate_mrac_cost(initial_guess, modelName, p, t_end, dt, x0, waypoints_ts, ...
                                             Am_x, Bm_x, P_x, Am_y, Bm_y, P_y, Am_z, Bm_z, Pz, weights, []);
disp('Baseline metrics:');
disp(baseline_metrics);
% --- Calculate Initial Cost (for final table) ---
initial_cost = weights.itae * 1 + ...
               weights.settling_time * 1 + ...
               weights.overshoot * 1 + ...
               weights.control_effort * 1 + ...
               weights.saturation_penalty * 1;
disp(['Baseline Weighted Cost: ', num2str(initial_cost)]);
% --- Create Objective Function Handle ---
objective_function = @(params) calculate_mrac_cost(params, modelName, p, t_end, dt, x0, waypoints_ts, ...
                                                 Am_x, Bm_x, P_x, Am_y, Bm_y, P_y, Am_z, Bm_z, Pz, ...
                                                 weights, baseline_metrics);

%% PART 6: RUN GRID SEARCH %% --- HEAVILY MODIFIED --- %%
% -------------------------------------------------------------------------
disp('Starting parameter grid search (this may take a long time)...');

% --- 1. Define the Grid of Values to Test ---
alpha_grid = [10, 25, 50, 75, 100];
beta_grid  = [1, 2, 5, 7, 10];
gamma_grid = [1, 2, 5, 7, 10];

% --- 2. Initialize Search Variables ---
best_cost = inf;
best_params_vec = [];
num_sims = length(alpha_grid) * length(beta_grid) * length(gamma_grid);
sim_count = 0;

% --- 3. Run the Nested Loops ---
for a = 1:length(alpha_grid)
    for b = 1:length(beta_grid)
        for g = 1:length(gamma_grid)
            sim_count = sim_count + 1;
            
            % Get current parameter set
            current_params = [alpha_grid(a); beta_grid(b); gamma_grid(g)];
            
            fprintf('--- Running Sim %d of %d: [a=%.1f, b=%.1f, g=%.1f] ---\n', ...
                    sim_count, num_sims, current_params(1), current_params(2), current_params(3));
            
            % Run the simulation and get the cost
            current_cost = objective_function(current_params);
            
            % Check if this is the best cost so far
            if current_cost < best_cost
                best_cost = current_cost;
                best_params_vec = current_params;
                fprintf('>>> New Best Cost Found: %.4f\n', best_cost);
            end
        end
    end
end

% --- 4. Assign Final Results ---
final_cost = best_cost;
best_params.alpha = best_params_vec(1);
best_params.beta  = best_params_vec(2);
best_params.gamma = best_params_vec(3);


%% PART 7: DISPLAY RESULTS 
% -------------------------------------------------------------------------
disp('--- GRID SEARCH COMPLETE ---');
disp(['Initial Cost: ', num2str(initial_cost)]);
disp(['Minimum Cost Found: ', num2str(final_cost)]);
disp('Best parameters found:');
fprintf('  alpha: %.4f\n', best_params.alpha);
fprintf('  beta:  %.4f\n', best_params.beta);
fprintf('  gamma: %.4f\n', best_params.gamma);

% --- Run simulation one last time with best params to get final metrics ---
disp('Running final simulation with optimized parameters...');
[~, final_metrics] = calculate_mrac_cost(best_params_vec, modelName, p, t_end, dt, x0, waypoints_ts, ...
                                             Am_x, Bm_x, P_x, Am_y, Bm_y, P_y, Am_z, Bm_z, Pz, ...
                                             weights, baseline_metrics);
                                         
% --- Create Data Analysis Table ---
disp('--- Data Analysis Results ---');
MetricNames = {
    'ITAE (Tracking)'
    'Gain Settling Time (s)'
    'Tilt Overshoot (rad)'
    'Control Effort (L2)'
    'Saturation Penalty'
    '--- TOTAL COST ---'
};
InitialValues = [
    baseline_metrics.itae
    baseline_metrics.settling_time
    baseline_metrics.overshoot
    baseline_metrics.control_effort
    baseline_metrics.saturation_penalty
    initial_cost
];
FinalValues = [
    final_metrics.itae
    final_metrics.settling_time
    final_metrics.overshoot
    final_metrics.control_effort
    final_metrics.saturation_penalty
    final_cost
];
% Calculate percentage change
PercentChange = ((FinalValues - InitialValues) ./ abs(InitialValues)) * 100;
% Create and display the table
T = table(InitialValues, FinalValues, PercentChange, 'RowNames', MetricNames);
disp(T);
disp('Note: A negative "PercentChange" indicates an improvement for that metric.');
% Close the Simulink model
close_system(modelName, 0);

%% UTILITY FUNCTIONS (Copied from main script)
function [Am, Bm, Cm, Dm] = create_reference_model(A,B)
    system_order = size(A, 1); 
    if system_order == 2
        disp('Designing 2th-order (Planar) Reference Model...');
        Ts_z = 3; 
        omega_z = 4 / Ts_z;
        zeta_z = 0.9;
        poles = roots([1, 2*zeta_z*omega_z, omega_z^2]);
        Km = place(A, B, poles);
        Am = A - B*Km;
        C_output_yaw = [1, 0]; 
        N = -1 / (C_output_yaw * inv(Am) * B);
        Bm = B * N;
        Cm = eye(2);
        Dm = zeros(2,1);
        disp('Second-order reference model created successfully.');

    elseif system_order == 4
        disp('Designing 4th-order (Planar) Reference Model...');
        Ts_position = 3;  
        Ts_balance_ratio = 3; 
        Ts_balance = Ts_position / Ts_balance_ratio;
        omega_np = 4 / (Ts_position);
        omega_nb = 4 / (Ts_balance); 
        zeta_p = 0.97;      
        zeta_b = 1.5;         
        p_slow = roots([1, 2*zeta_p*omega_np, omega_np^2]);
        p_fast = roots([1, 2*zeta_b*omega_nb, omega_nb^2]);
        poles = [p_slow; p_fast]; 
        Km = place(A, B, poles);
        Am = A - B*Km;
        C_output_phi = [0, 0, 1, 0]; 
        N = -1 / (C_output_phi * inv(Am) * B);
        Bm = B * N;
        Cm = eye(4);
        Dm = zeros(4,1);
        disp('Fourth-order reference model created successfully.');
    end
end