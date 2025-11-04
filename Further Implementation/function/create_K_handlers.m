function [K_front_func, K_sag_func, K_yaw_func] = create_K_handlers(Afront, Bfront, Asag, Bsag, Ayaw, Byaw)
% CREATE_K_FROM_LPV Creates LPV state-feedback gain handles from LPV models.
%
% This function takes existing LPV model function handles (A and B for each
% plane) and performs the offline analysis to generate the corresponding
% gain function handles (K) for each plane of motion.
%
% INPUTS:
%   Afront, Bfront - Function handles for the Frontal (Y-Z) plane model.
%   Asag, Bsag   - Function handles for the Sagittal (X-Z) plane model.
%   Ayaw, Byaw     - Function handles for the Yaw (X-Y) plane model.
%
% OUTPUTS:
%   K_sag_func   - A function handle for the Sagittal gain, K = K_sag_func(th_y).
%   K_front_func - A function handle for the Frontal gain, K = K_front_func(th_x).
%   K_yaw_func   - A function handle for the Yaw gain, K = K_yaw_func().

%% Step 1: Define Desired Performance (Poles) for each Subsystem

disp('Defining desired closed-loop performance...');

% --- TUNABLE PARAMETERS ---
% These parameters define how you want your robot to behave.
% (You can adjust these values as needed)
Ts_velocity = 2.0;
Ts_balance_ratio = 4;
Ts_balance = Ts_velocity / Ts_balance_ratio; 
zeta_v = 1.2;
zeta_b = 0.9;
Ts_yaw = 1;
zeta_yaw = 0.9;
% --- END TUNABLE PARAMETERS ---

% Calculate the desired pole locations
omega_nv = 4 / Ts_velocity;
omega_nb = 4 / Ts_balance;
p_slow = roots([1, 2*zeta_v*omega_nv, omega_nv^2]);
p_fast = roots([1, 2*zeta_b*omega_nb, omega_nb^2]);
poles_4_state = [p_slow; p_fast];

omega_yaw = 4 / Ts_yaw;
poles_2_state = roots([1, 2*zeta_yaw*omega_yaw, omega_yaw^2]);

%% Step 2: Calculate K at Discrete Operating Points

disp('Calculating K matrices at discrete points...');

theta_points_deg = -15:3:15;
theta_points_rad = deg2rad(theta_points_deg);
num_points = length(theta_points_rad);

K_sag_models = cell(1, num_points);
K_front_models = cell(1, num_points);

for i = 1:num_points
    th_rad = theta_points_rad(i);
    
    % Calculate K for the Sagittal plane using the provided function handles
    K_sag_models{i} = place(Asag(th_rad), Bsag(th_rad), poles_4_state);
    
    % Calculate K for the Frontal plane using the provided function handles
    K_front_models{i} = place(Afront(th_rad), Bfront(th_rad), poles_4_state);
end

% For Yaw, A and B are constant, so K is also constant.
K_yaw_const = place(Ayaw(), Byaw(), poles_2_state);

%% Step 3: Fit Polynomials to the Gain Elements

disp('Fitting polynomials to gain schedules...');
poly_order = 4;

% Fit Sagittal Gains
coeffs_sag = cell(1, 4);
for element = 1:4
    data_points = cellfun(@(m) m(1, element), K_sag_models);
    coeffs_sag{element} = polyfit(theta_points_rad, data_points, poly_order);
end

% Fit Frontal Gains
coeffs_front = cell(1, 4);
for element = 1:4
    data_points = cellfun(@(m) m(1, element), K_front_models);
    coeffs_front{element} = polyfit(theta_points_rad, data_points, poly_order);
end

%% Step 4: Create Final, Callable Function Handles

disp('Creating final function handles for K...');

K_sag_func = @(th_y_rad) get_scheduled_K(th_y_rad, coeffs_sag);
K_front_func = @(th_x_rad) get_scheduled_K(th_x_rad, coeffs_front);
K_yaw_func = @() K_yaw_const;

disp('K function handlers created successfully.');

end

% --- NESTED HELPER FUNCTION ---
function K_current = get_scheduled_K(theta_rad, coeffs)
    K_current = zeros(1, 4);
    for element = 1:4
        K_current(element) = polyval(coeffs{element}, theta_rad);
    end
end