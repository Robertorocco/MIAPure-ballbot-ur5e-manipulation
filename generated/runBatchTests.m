% Filename: runBatchTests_EquilibriumComp.m
clear; clc; close all;
addpath('function'); 
%% --- Simulation Parameters ---
t_end = 10;
dt = 0.0005;
num_tests = 3;
rng('shuffle');
%% --- Define Parameter Ranges for Randomized Search ---
disp('Defining parameter search space...');
% ... (definizione dei range invariata) ...
%FIRST EXPERIMENT
% param_ranges.Kp_pos   = [0.1, 2];
% param_ranges.Kp_inner = [0.1, 3];
% param_ranges.Ki_inner = [0.1, 5];
% param_ranges.Q1 = [1, 20];
% param_ranges.Q2 = [1, 20];
% param_ranges.Q3 = [0.1, 2];
% param_ranges.Q4 = [0.1, 2];
% R_val = 1;
% param_ranges.initial_pos_xy = [-0.5, 0.5]; 
% param_ranges.initial_tilt_rad = [-0.1, 0.1]; %(5.7 deg)
%Second experiment
param_ranges.Kp_pos   = [0.5, 2];
param_ranges.Kp_inner = [0.5, 2];
param_ranges.Ki_inner = [2.5, 5];
param_ranges.Q1 = [1, 50];
param_ranges.Q2 = [1, 10];
param_ranges.Q3 = [10, 30];
param_ranges.Q4 = [1, 10];
R_val = 1;
param_ranges.initial_pos_xy = [-0.1, 0.1]; 
param_ranges.initial_tilt_rad = [-0.07, 0.07]; %(5.7 deg)
param_ranges.k_op = [0.4, 1]; %% <-- ADDED:  Define k_op range

% 1. Define the pose of the VIRTUAL, fixed base (Unchanged)
%modelName = 'scheme_batch_regulation';
modelName = 'scheme_batch_op_regulation';
manipulator_sim; 
p_virtual_base = [0; 0; 0.50];
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
%% --- Generate Test Parameter Sets ---
% ... (generazione parametri invariata) ...
test_params = cell(num_tests, 1);
rng('default');
for i = 1:num_tests
    gains.Kp_pos   = param_ranges.Kp_pos(1)   + rand() * (param_ranges.Kp_pos(2)   - param_ranges.Kp_pos(1));
    gains.Kp_inner = param_ranges.Kp_inner(1) + rand() * (param_ranges.Kp_inner(2) - param_ranges.Kp_inner(1));
    gains.Ki_inner = param_ranges.Ki_inner(1) + rand() * (param_ranges.Ki_inner(2) - param_ranges.Ki_inner(1));
    gains.k_op     = param_ranges.k_op(1)     + rand() * (param_ranges.k_op(2)     - param_ranges.k_op(1)); %% <-- ADDED: Generate random k_op
    
    q1 = param_ranges.Q1(1) + rand() * (param_ranges.Q1(2) - param_ranges.Q1(1));
    q2 = param_ranges.Q2(1) + rand() * (param_ranges.Q2(2) - param_ranges.Q2(1));
    q3 = param_ranges.Q3(1) + rand() * (param_ranges.Q3(2) - param_ranges.Q3(1));
    q4 = param_ranges.Q4(1) + rand() * (param_ranges.Q4(2) - param_ranges.Q4(1));
    Q_diag = [q1, q2, q3, q4]; 
    ic.pos_x = param_ranges.initial_pos_xy(1) + rand() * (param_ranges.initial_pos_xy(2) - param_ranges.initial_pos_xy(1));
    ic.pos_y = param_ranges.initial_pos_xy(1) + rand() * (param_ranges.initial_pos_xy(2) - param_ranges.initial_pos_xy(1));
    ic.tilt_roll  = param_ranges.initial_tilt_rad(1) + rand() * (param_ranges.initial_tilt_rad(2) - param_ranges.initial_tilt_rad(1));
    ic.tilt_pitch = param_ranges.initial_tilt_rad(1) + rand() * (param_ranges.initial_tilt_rad(2) - param_ranges.initial_tilt_rad(1));
    test_params{i}.gains = gains;
    test_params{i}.Q_diag = Q_diag;
    test_params{i}.R = R_val;
    test_params{i}.initial_conditions = ic;
end
%% --- Run Batch Simulations ---
% --- MODIFICATO: Aggiunta la colonna k_op e aumentata Size a 18 ---
results_table = table('Size', [num_tests, 18], ...  %% <-- MODIFIED: Size 17 -> 18
    'VariableTypes', {'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double'}, ... %% <-- MODIFIED: Added 'double'
    'VariableNames', {'TestNum', 'Kp_pos', 'Kp_inner', 'Ki_inner', 'k_op', 'Q1', 'Q2', 'Q3', 'Q4', 'StartX', 'StartY', 'StartRoll', 'StartPitch', 'RMSE', 'MaxDist', 'ConvTime', 'OP_RMSE', 'Cost'}); %% <-- MODIFIED: Added 'k_op'

disp('Starting batch simulation tests...');
for i = 1:num_tests
    fprintf('\n--- Running Test %d of %d ---\n', i, num_tests);
    current_params = test_params{i};
    
    % This line now implicitly passes 'gains.k_op' to the function
    try
    metrics = runBallbotSimulation(op_space_pose_trajectory, current_params.Q_diag, current_params.R, current_params.gains, t_end, dt, false, current_params.initial_conditions);
    catch ME
        fprintf('    --> ERROR during simulation: %s\n', ME.message);
        metrics.rmse = Inf; metrics.max_dist = Inf; metrics.conv_time = Inf;
        metrics.op_rmse = Inf; % Ensure op_rmse is also Inf on failure
    end
    
    %% --- Salva tutti i parametri e le metriche GREZZE ---
    results_table.TestNum(i) = i;
    results_table.Kp_pos(i)   = current_params.gains.Kp_pos;
    results_table.Kp_inner(i) = current_params.gains.Kp_inner;
    results_table.Ki_inner(i) = current_params.gains.Ki_inner;
    results_table.k_op(i)     = current_params.gains.k_op; %% <-- ADDED: Save k_op to table
    results_table.Q1(i) = current_params.Q_diag(1);
    results_table.Q2(i) = current_params.Q_diag(2);
    results_table.Q3(i) = current_params.Q_diag(3);
    results_table.Q4(i) = current_params.Q_diag(4);
    results_table.StartX(i) = current_params.initial_conditions.pos_x;
    results_table.StartY(i) = current_params.initial_conditions.pos_y;
    results_table.StartRoll(i)  = current_params.initial_conditions.tilt_roll;
    results_table.StartPitch(i) = current_params.initial_conditions.tilt_pitch;
    results_table.RMSE(i) = metrics.rmse;
    results_table.MaxDist(i) = metrics.max_dist;
    results_table.ConvTime(i) = metrics.conv_time;
    results_table.OP_RMSE(i) = metrics.op_rmse;
end
disp('All simulations complete.');
%% --- NUOVA SEZIONE: Analisi Performance e Calcolo Funzione di Costo ---
disp('Calculating normalized cost function...');
% Filtra solo i test che sono andati a buon fine
valid_results = results_table(isfinite(results_table.RMSE), :);
failed_indices = find(isinf(results_table.RMSE));
if ~isempty(valid_results)
    % 1. Normalizzazione rispetto alle condizioni iniziali
    initial_dist = sqrt(valid_results.StartX.^2 + valid_results.StartY.^2);
    
    % RMSE normalizzato (senza unità)
    norm_rmse = valid_results.RMSE ./ (initial_dist + 1e-6);
    
    % Overshoot normalizzato (distanza extra percorsa rispetto al raggio iniziale)
    overshoot = max(0, valid_results.MaxDist - initial_dist);
    norm_overshoot = overshoot ./ (initial_dist + 1e-6);
    
    % Tempo di convergenza (già in secondi)
    norm_conv_time = valid_results.ConvTime;
    % 2. Scalatura Min-Max per rendere le metriche confrontabili (range [0, 1])
    scale = @(x) (x - min(x)) ./ (max(x) - min(x) + 1e-6);
    
    scaled_rmse = scale(norm_rmse);
    scaled_overshoot = scale(norm_overshoot);
    scaled_conv_time = scale(norm_conv_time);
    
    % 3. Funzione di Costo: somma pesata (qui pesi uguali a 1)
    cost = scaled_rmse + scaled_overshoot + scaled_conv_time;
    
    % Inserisci il costo calcolato nella tabella
    valid_results.Cost = cost;
    results_table.Cost = Inf(height(results_table), 1); % Imposta Inf di default
    results_table(isfinite(results_table.RMSE), :) = valid_results; % Ri-assegna i risultati validi
else
    results_table.Cost = Inf(height(results_table), 1);
end
%% --- Display Results and Generate LaTeX Table ---
results_table = sortrows(results_table, 'Cost', 'ascend'); % Sort by Cost
disp('--- Final Results (Sorted by Best Cost) ---');
% MODIFIED: Added k_op to the console display
disp(results_table(:, {'TestNum', 'Kp_pos', 'Kp_inner', 'Ki_inner', 'k_op', 'RMSE', 'MaxDist', 'ConvTime', 'OP_RMSE', 'Cost'})); %% <-- MODIFIED

fprintf('\n\n--- LaTeX Code for Results Table (Summarized) ---\n');
fprintf('\\begin{table}[h!]\n');
fprintf('\\centering\n');
fprintf('\\caption{Controller Performance Ranking by Normalized Cost}\n');
fprintf('\\label{tab:cost_function_results}\n');
fprintf('\\resizebox{\\textwidth}{!}{\n');
% MODIFIED: Added one more column specifier 'c' (10 -> 11)
fprintf('\\begin{tabular}{|c|c|c|c|c|c|c|c|c|c|c|}\n'); %% <-- MODIFIED
fprintf('\\hline\n');
% MODIFIED: Added K_op to the header
fprintf('Rank & Test & $K_{p,pos}$ & $K_{p,in}$ & $K_{i,in}$ & \\textbf{$K_{op}$} & RMSE (m) & Max Dist (m) & Conv Time (s) & \\textbf{OP\\_RMSE} (m) & \\textbf{Cost} \\\\\n'); %% <-- MODIFIED
fprintf('\\hline\n');
for i = 1:min(height(results_table), 20)
    row = results_table(i, :);
    
    cost_string = '';
    if isinf(row.Cost)
        cost_string = '\textcolor{red}{Failed}';
    else
        cost_string = sprintf('\\textbf{%.4f}', row.Cost);
    end
    
    % --- MODIFIED: Added format specifier (%.2f) and variable (row.k_op) ---
    fprintf('%d & %d & %.2f & %.2f & %.2f & %.2f & %.3f & %.3f & %.2f & %.4f & %s \\\\\n', ... %% <-- MODIFIED
        i, ...
        row.TestNum, ...
        row.Kp_pos, ...
        row.Kp_inner, ...
        row.Ki_inner, ...
        row.k_op, ... %% <-- ADDED
        row.RMSE, ...
        row.MaxDist, ...
        row.ConvTime, ...
        row.OP_RMSE, ...
        cost_string);
end
fprintf('\\hline\n');
fprintf('\\end{tabular}}\n');
fprintf('\\end{table}\n\n');