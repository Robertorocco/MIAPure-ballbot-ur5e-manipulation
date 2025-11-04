function [A, B] = linearizeBallbotModel_MRAC(p, x_eq, u_eq)
% LINEARIZEBALLBOTMODEL Computes the linearized state-space matrices (A and B)
% for the ballbot model around a given equilibrium point.
%
% INPUTS:
%   p    - A struct containing the numerical values for the robot's parameters.
%   x_eq - The state vector [10x1] at the equilibrium point.
%   u_eq - The input vector [3x1] at the equilibrium point.
%
% OUTPUTS:
%   A - The linearized state matrix (10x10).
%   B - The linearized input matrix (10x3).
% =========================================================================

%% --- PART 1: DEFINE SYMBOLIC DYNAMICS ---

% --- Define Symbolic Variables ---
syms t
syms theta_y(t) phi_y(t) theta_x(t) phi_x(t)
syms dot_theta_y dot_phi_y ddot_theta_y ddot_phi_y
syms dot_theta_x dot_phi_x ddot_theta_x ddot_phi_x
syms dot_theta_z ddot_theta_z % For yaw
syms tau_y tau_x tau_z b_theta b_phi fz
syms m_U I_Uy I_Ux Iz l_U m_S I_S r_S g h_app
syms th_y ph_y th_x ph_x th_z


% --- X-Z Plane (Roll) Dynamics ---
T_Sy = 0.5 * m_S * (r_S * diff(phi_y,t))^2 + 0.5 * I_S * diff(phi_y,t)^2;
v_Uy_x = r_S * diff(phi_y,t) + l_U * cos(theta_y) * diff(theta_y,t);
v_Uy_z = -l_U * sin(theta_y) * diff(theta_y,t);
T_Uy = 0.5 * m_U * (v_Uy_x^2 + v_Uy_z^2) + 0.5 * I_Uy * diff(theta_y,t)^2;
T_y = T_Sy + T_Uy;
V_y = m_U * g * l_U * cos(theta_y);
L_y = T_y - V_y;
dLd_dot_theta_y = diff(L_y, diff(theta_y,t));
ddt_dLd_dot_theta_y = diff(dLd_dot_theta_y, t);
dLd_theta_y = diff(L_y, theta_y);

Q_theta_y = -tau_y -b_theta*diff(theta_y,t);

EoM1_y = ddt_dLd_dot_theta_y - dLd_theta_y == Q_theta_y;
dLd_dot_phi_y = diff(L_y, diff(phi_y,t));
ddt_dLd_dot_phi_y = diff(dLd_dot_phi_y, t);
dLd_phi_y = diff(L_y, phi_y);

Q_phi_y = tau_y - b_phi * diff(phi_y,t) ;

EoM2_y = ddt_dLd_dot_phi_y - dLd_phi_y == Q_phi_y;
vars_y = [diff(theta_y,t,t), diff(phi_y,t,t), diff(theta_y,t), diff(phi_y,t), theta_y(t)];
subs_vars_y = [ddot_theta_y, ddot_phi_y, dot_theta_y, dot_phi_y, th_y];
EoM1_y_sub = subs(EoM1_y, vars_y, subs_vars_y);
EoM2_y_sub = subs(EoM2_y, vars_y, subs_vars_y);
sol_y = solve([EoM1_y_sub, EoM2_y_sub], [ddot_theta_y, ddot_phi_y]);
ddot_theta_y_expr = sol_y.ddot_theta_y;
ddot_phi_y_expr = sol_y.ddot_phi_y;

% --- Y-Z Plane (Pitch) Dynamics ---
T_Sx = 0.5 * m_S * (r_S * diff(phi_x,t))^2 + 0.5 * I_S * diff(phi_x,t)^2;
v_Ux_y = -r_S * diff(phi_x,t) - l_U * cos(theta_x) * diff(theta_x,t);
v_Ux_z = -l_U * sin(theta_x) * diff(theta_x,t);
T_Ux = 0.5 * m_U * (v_Ux_y^2 + v_Ux_z^2) + 0.5 * I_Ux * diff(theta_x,t)^2;
T_x = T_Sx + T_Ux;
V_x = m_U * g * l_U * cos(theta_x);
L_x = T_x - V_x;
dLd_dot_theta_x = diff(L_x, diff(theta_x,t));
ddt_dLd_dot_theta_x = diff(dLd_dot_theta_x, t);
dLd_theta_x = diff(L_x, theta_x);

Q_theta_x = -tau_x -b_theta*diff(theta_x,t);

EoM1_x = ddt_dLd_dot_theta_x - dLd_theta_x == Q_theta_x;
dLd_dot_phi_x = diff(L_x, diff(phi_x,t));
ddt_dLd_dot_phi_x = diff(dLd_dot_phi_x, t);
dLd_phi_x = diff(L_x, phi_x);

Q_phi_x = tau_x - b_phi * diff(phi_x,t);

EoM2_x = ddt_dLd_dot_phi_x - dLd_phi_x == Q_phi_x;
vars_x = [diff(theta_x,t,t), diff(phi_x,t,t), diff(theta_x,t), diff(phi_x,t), theta_x(t)];
subs_vars_x = [ddot_theta_x, ddot_phi_x, dot_theta_x, dot_phi_x, th_x];
EoM1_x_sub = subs(EoM1_x, vars_x, subs_vars_x);
EoM2_x_sub = subs(EoM2_x, vars_x, subs_vars_x);
sol_x = solve([EoM1_x_sub, EoM2_x_sub], [ddot_theta_x, ddot_phi_x]);
ddot_theta_x_expr = sol_x.ddot_theta_x;
ddot_phi_x_expr = sol_x.ddot_phi_x;

% --- Yaw Dynamics ---
ddot_theta_z_expr = (tau_z - fz * dot_theta_z) / Iz;


%% --- PART 2: COMPUTE JACOBIANS ---

% Define the symbolic state vector 'x' and input vector 'u'
x_sym = [th_x; dot_theta_x; ph_x; dot_phi_x; th_y; dot_theta_y; ph_y; dot_phi_y; th_z; dot_theta_z;];
u_sym = [tau_x; tau_y; tau_z];
% Define the symbolic state derivative vector f(x,u)
f_sym = [
    dot_theta_x;
    ddot_theta_x_expr;
    dot_phi_x;
    ddot_phi_x_expr;
    dot_theta_y;
    ddot_theta_y_expr;
    dot_phi_y;
    ddot_phi_y_expr;
    dot_theta_z;
    ddot_theta_z_expr;
];

% Calculate the Jacobians
A_sym = jacobian(f_sym, x_sym);
B_sym = jacobian(f_sym, u_sym);
%% --- PART 3: EVALUATE AT EQUILIBRIUM POINT ---

% Substitute the equilibrium values (passed as arguments) into the symbolic Jacobians
A_eval = subs(A_sym, [x_sym; u_sym], [x_eq; u_eq]);
B_eval = subs(B_sym, [x_sym; u_sym], [x_eq; u_eq]);
%% --- PART 4: CONVERT TO NUMERICAL MATRICES ---

% Create a list of symbolic parameters and their corresponding numerical values
param_syms = [m_U, I_Ux, I_Uy, Iz, l_U, m_S, I_S, r_S g, b_theta, b_phi, fz, h_app];
param_vals = [p.m_U, p.I_Ux, p.I_Uy, p.Iz, p.l_U, p.m_S, p.I_S, p.r_S, p.g, p.b_theta, p.b_phi, p.fz, p.h_app];

% Substitute the numerical parameter values into the evaluated Jacobians
A_num_sym = subs(A_eval, param_syms, param_vals);
B_num_sym = subs(B_eval, param_syms, param_vals);

% Convert the final symbolic matrices to standard double-precision matrices
A = double(A_num_sym);
B = double(B_num_sym);

end
