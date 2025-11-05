% --- MODIFICATO: La funzione ora restituisce una struct con più metriche ---
function performance_metrics = runBallbotSimulation(op_space_pose_trajectory, Q_diag, R_val, gains, t_end, dt, show_plots, initial_conditions)
% OUTPUT:
%   performance_metrics - Struct con i campi: rmse, max_dist, conv_time, op_rmse
%% PART 1 & 2: SYSTEM PARAMETERS & LQR DESIGN
% ... (nessuna modifica in queste sezioni) ...
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
p.h_app = p.h_U - p.r_S;
x_eq = zeros(10, 1);
u_eq = zeros(8, 1);
[A_full, B_full] = linearizeBallbotModel(p, x_eq, u_eq);
pitch_states_idx = [2, 5, 7, 10];
pitch_input_idx = 2;
A = A_full(pitch_states_idx, pitch_states_idx);
B = B_full(pitch_states_idx, pitch_input_idx);
Q = diag(Q_diag); 
R = R_val;
performance_metrics.op_rmse = Inf;
try
    [K, ~, ~] = lqr(A, B, Q, R);
catch ME
    warning('LQR failed. Assigning Inf to metrics. Error: %s', ME.message);
    performance_metrics.rmse = Inf;
    performance_metrics.max_dist = Inf;
    performance_metrics.conv_time = Inf;
    performance_metrics.op_rmse = Inf; % <-- FIX (Questa riga è corretta)
    return;
end
%% PART 3 & 4: SIMULATION SETUP & EXECUTION
% ... (nessuna modifica alla logica di setup e avvio della simulazione) ...
goal_pos = [0, 0];
waypoints = goal_pos;
start_pos = [initial_conditions.pos_x, initial_conditions.pos_y];
obstacles_def = [];
map_scale = 1;
x0 = zeros(10, 1);
x0(1) = initial_conditions.tilt_roll;
x0(2) = initial_conditions.tilt_pitch;
x0(5) = start_pos(1) / p.r_S;
x0(4) = -start_pos(2) / p.r_S;
control_params.Kp_pos = gains.Kp_pos;
control_params.Kp_inner = gains.Kp_inner;
control_params.Ki_inner = gains.Ki_inner;
control_params.k_op = gains.k_op; % <-- MODIFICA 1: Aggiunto k_op
control_params.K = K;
control_params.A_lqr = A;
control_params.B_lqr = B;

modelName = 'scheme_batch_op_regulation'; % <-- MODIFICA 2: Corretto il nome del modello

manipulator_sim;
try
    % pulisco eventuali variabili residue per evitare riutilizzo
    if exist('simOut', 'var')
        clear simOut
    end
    warning('off', 'Simulink:Commands:LoadingOlderModel');
    % Ora 'op_space_pose_trajectory' e 'control_params' (con k_op) 
    % sono nello workspace 'current' della funzione e Simulink li vedrà.
    simOut = sim(modelName, 'SrcWorkspace', 'current', 'StopTime', num2str(t_end), 'FixedStep', num2str(dt));
    warning('on', 'Simulink:Commands:LoadingOlderModel');
    sim_failed = false;
catch ME_sim
    % La simulazione non è partita / è fallita: assegna Inf e ritorna
    warning('Simulazione fallita: %s', ME_sim.message);
    performance_metrics.rmse = Inf;
    performance_metrics.max_dist = Inf;
    performance_metrics.conv_time = t_end;
    performance_metrics.op_rmse = Inf;
    return;
end
%% PART 5: DATA PROCESSING AND METRICS CALCULATION
t_full = simOut.tout;
% --- CORREZIONE: Trasponi la matrice dei risultati ---
% Estrai i dati come prima, ottenendo una matrice [tempo x stati]
x_full_transposed = squeeze(permute(simOut.x_sim.Data, [1, 3, 2]));
% Ora trasponila per avere la forma corretta: [stati x tempo]
x_full = x_full_transposed'; 
% Da qui in poi, il resto del tuo codice funzionerà perfettamente
pos_x = p.r_S * x_full(5, :); 
pos_y = -p.r_S * x_full(4, :);
% --- Calcolo delle 3 metriche di performance ---
dist_from_origin = sqrt(pos_x.^2 + pos_y.^2);
performance_metrics.rmse = sqrt(mean(dist_from_origin.^2));
performance_metrics.max_dist = max(dist_from_origin);
% --- Calcolo del tempo di convergenza (ora funzionante) ---
conv_accuracy = 0.01;
conv_duration = 1.0;
conv_steps = round(conv_duration / dt);
is_converged = false;
for i = 1:(length(dist_from_origin) - conv_steps)
    window_indices = i:(i + conv_steps);
    if all(dist_from_origin(window_indices) < conv_accuracy)
        performance_metrics.conv_time = t_full(i);
        is_converged = true;
        break;
    end
end
if ~is_converged
    performance_metrics.conv_time = t_end;
end

% --- NUOVO: Calcola il Root Mean Square Error dello Spazio Operativo ---
% Assicurati che il tuo modello 'scheme_batch_op_regulation' salvi 
% l'errore in una variabile 'To Workspace' chiamata 'ErrorPos'
    error_pos_history = simOut.ErrorPos.Data;
    
    % Il formato dei dati è probabilmente 3x1xN, quindi lo "spremiamo" e lo trasponiamo in Nx3
    error_pos_matrix = squeeze(error_pos_history)';
    
    % Calcola la magnitudine (norma Euclidea) del vettore di errore per ogni istante
    error_magnitudes = vecnorm(error_pos_matrix, 2, 2);
    
    % Calcola la radice della media dei quadrati di queste magnitudini
    performance_metrics.op_rmse = sqrt(mean(error_magnitudes.^2));

end