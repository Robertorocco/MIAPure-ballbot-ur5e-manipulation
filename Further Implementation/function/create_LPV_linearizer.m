function [A_roll, B_roll, A_pitch, B_pitch, A_spin, B_spin] = create_LPV_linearizer(p)
    % Creates decoupled LPV models for each plane of motion.
    % (This version is corrected to avoid MATLAB's invalid array indexing error).
    
    % ... (PARTS 1, 2, 3, and 4 are completely unchanged from the previous version) ...
    % --- They perform the full symbolic linearization and create the 
    % --- full 10x10 A_full_func and 10x3 B_full_func handles.
    syms t; syms theta_y(t) phi_y(t) theta_x(t) phi_x(t); syms dot_theta_y dot_phi_y ddot_theta_y ddot_phi_y; syms dot_theta_x dot_phi_x ddot_theta_x ddot_phi_x; syms dot_theta_z ddot_theta_z; syms tau_y tau_x tau_z b_theta b_phi fz; syms m_U I_Uy I_Ux Iz l_U m_S I_S r_S g h_app; syms th_y ph_y th_x ph_x th_z; syms F_px F_py F_pz tau_px tau_py tau_pz;
    T_Sy = 0.5 * m_S * (r_S * diff(phi_y,t))^2 + 0.5 * I_S * diff(phi_y,t)^2; v_Uy_x = r_S * diff(phi_y,t) + l_U * cos(theta_y) * diff(theta_y,t); v_Uy_z = -l_U * sin(theta_y) * diff(theta_y,t); T_Uy = 0.5 * m_U * (v_Uy_x^2 + v_Uy_z^2) + 0.5 * I_Uy * diff(theta_y,t)^2; T_y = T_Sy + T_Uy; V_y = m_U * g * l_U * cos(theta_y); L_y = T_y - V_y; dLd_dot_theta_y = diff(L_y, diff(theta_y,t)); ddt_dLd_dot_theta_y = diff(dLd_dot_theta_y, t); dLd_theta_y = diff(L_y, theta_y); Q_theta_y = -tau_y -b_theta*diff(theta_y,t) + h_app*(F_px*cos(theta_y)-F_pz*sin(theta_y)) + tau_py; EoM1_y = ddt_dLd_dot_theta_y - dLd_theta_y == Q_theta_y; dLd_dot_phi_y = diff(L_y, diff(phi_y,t)); ddt_dLd_dot_phi_y = diff(dLd_dot_phi_y, t); dLd_phi_y = diff(L_y, phi_y); Q_phi_y = tau_y - b_phi * diff(phi_y,t) + r_S * F_px; EoM2_y = ddt_dLd_dot_phi_y - dLd_phi_y == Q_phi_y;
    vars_y = [diff(theta_y,t,t), diff(phi_y,t,t), diff(theta_y,t), diff(phi_y,t), theta_y(t)]; subs_vars_y = [ddot_theta_y, ddot_phi_y, dot_theta_y, dot_phi_y, th_y]; EoM1_y_sub = subs(EoM1_y, vars_y, subs_vars_y); EoM2_y_sub = subs(EoM2_y, vars_y, subs_vars_y); sol_y = solve([EoM1_y_sub, EoM2_y_sub], [ddot_theta_y, ddot_phi_y]); ddot_theta_y_expr = sol_y.ddot_theta_y; ddot_phi_y_expr = sol_y.ddot_phi_y;
    T_Sx = 0.5 * m_S * (r_S * diff(phi_x,t))^2 + 0.5 * I_S * diff(phi_x,t)^2; v_Ux_y = -r_S * diff(phi_x,t) - l_U * cos(theta_x) * diff(theta_x,t); v_Ux_z = -l_U * sin(theta_x) * diff(theta_x,t); T_Ux = 0.5 * m_U * (v_Ux_y^2 + v_Ux_z^2) + 0.5 * I_Ux * diff(theta_x,t)^2; T_x = T_Sx + T_Ux; V_x = m_U * g * l_U * cos(theta_x); L_x = T_x - V_x; dLd_dot_theta_x = diff(L_x, diff(theta_x,t)); ddt_dLd_dot_theta_x = diff(dLd_dot_theta_x, t); dLd_theta_x = diff(L_x, theta_x); Q_theta_x = -tau_x -b_theta*diff(theta_x,t) + h_app*(-F_py*cos(theta_x)-F_pz*sin(theta_x)) + tau_px; EoM1_x = ddt_dLd_dot_theta_x - dLd_theta_x == Q_theta_x; dLd_dot_phi_x = diff(L_x, diff(phi_x,t)); ddt_dLd_dot_phi_x = diff(dLd_dot_phi_x, t); dLd_phi_x = diff(L_x, phi_x); Q_phi_x = tau_x - b_phi * diff(phi_x,t) - r_S * F_py; EoM2_x = ddt_dLd_dot_phi_x - dLd_phi_x == Q_phi_x;
    vars_x = [diff(theta_x,t,t), diff(phi_x,t,t), diff(theta_x,t), diff(phi_x,t), theta_x(t)]; subs_vars_x = [ddot_theta_x, ddot_phi_x, dot_theta_x, dot_phi_x, th_x]; EoM1_x_sub = subs(EoM1_x, vars_x, subs_vars_x); EoM2_x_sub = subs(EoM2_x, vars_x, subs_vars_x); sol_x = solve([EoM1_x_sub, EoM2_x_sub], [ddot_theta_x, ddot_phi_x]); ddot_theta_x_expr = sol_x.ddot_theta_x; ddot_phi_x_expr = sol_x.ddot_phi_x;
    ddot_theta_z_expr = (tau_z + tau_pz - fz * dot_theta_z) / Iz; x_sym = [th_x; dot_theta_x; ph_x; dot_phi_x; th_y; dot_theta_y; ph_y; dot_phi_y; th_z; dot_theta_z]; u_sym_control = [tau_x; tau_y; tau_z]; f_sym = [dot_theta_x; ddot_theta_x_expr; dot_phi_x; ddot_phi_x_expr; dot_theta_y; ddot_theta_y_expr; dot_phi_y; ddot_phi_y_expr; dot_theta_z; ddot_theta_z_expr;];
    A_sym_full = jacobian(f_sym, x_sym); B_sym_full = jacobian(f_sym, u_sym_control);
    x_op_sym = [th_x; 0; 0; 0; th_y; 0; 0; 0; 0; 0]; u_op_sym = [0; 0; 0]; disturbance_op_sym = [F_px; F_py; F_pz; tau_px; tau_py; tau_pz]; A_sym_scheduled = subs(A_sym_full, [x_sym; u_sym_control; disturbance_op_sym], [x_op_sym; u_op_sym; zeros(6,1)]); B_sym_scheduled = subs(B_sym_full, [x_sym; u_sym_control; disturbance_op_sym], [x_op_sym; u_op_sym; zeros(6,1)]);
    param_syms = [m_U, I_Ux, I_Uy, Iz, l_U, m_S, I_S, r_S g, b_theta, b_phi, fz, h_app]; param_vals = [p.m_U, p.I_Ux, p.I_Uy, p.Iz, p.l_U, p.m_S, p.I_S, p.r_S, p.g, p.b_theta, p.b_phi, p.fz, p.h_app]; A_final_sym = subs(A_sym_scheduled, param_syms, param_vals); B_final_sym = subs(B_sym_scheduled, param_syms, param_vals);
    A_full_func = matlabFunction(A_final_sym, 'Vars', {th_x, th_y}); B_full_func = matlabFunction(B_final_sym, 'Vars', {th_x, th_y});

    %% --- PART 5: DECOUPLE INTO SUBSYSTEMS --- *** CORRECTED SECTION ***

    disp('Decoupling LPV models into subsystems...');

    % Define the indices for each subsystem
    idx_front = 1:4;
    idx_sag   = 5:8;
    idx_yaw   = 9:10;
    idx_tau_x = 1;
    idx_tau_y = 2;
    idx_tau_z = 3;

    % --- Create Anonymous Function Handles for each Subsystem ---
    % We use a helper function 'getSubMatrix' to avoid the syntax error.

    % Sagittal (Pitch) Model: A(th_y), B(th_y)
    A_pitch = @(th_y_rad) getSubMatrix(A_full_func(0, th_y_rad), idx_sag, idx_sag);
    B_pitch = @(th_y_rad) getSubMatrix(B_full_func(0, th_y_rad), idx_sag, idx_tau_y);

    % Frontal (Roll) Model: A(th_x), B(th_x)
    A_roll = @(th_x_rad) getSubMatrix(A_full_func(th_x_rad, 0), idx_front, idx_front);
    B_roll = @(th_x_rad) getSubMatrix(B_full_func(th_x_rad, 0), idx_front, idx_tau_x);

    % Yaw Model: A, B (These are constant)
    A_yaw_const = getSubMatrix(A_full_func(0, 0), idx_yaw, idx_yaw);
    B_yaw_const = getSubMatrix(B_full_func(0, 0), idx_yaw, idx_tau_z);
    A_spin = @() A_yaw_const;
    B_spin = @() B_yaw_const;

    disp('Decoupled function handles created successfully.');

end

% --- NESTED HELPER FUNCTION ---
function M_sub = getSubMatrix(M_full, rows, cols)
    % This helper function correctly separates the function call from the indexing.
    M_sub = M_full(rows, cols);
end