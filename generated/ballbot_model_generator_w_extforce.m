%% SCRIPT TO AUTOMATICALLY DERIVE AND GENERATE BALLBOT EOMs (Corrected)
% -------------------------------------------------------------------------
% This script performs the following steps:
% 1.  Derives the equations of motion for the ballbot in both the Y-Z
%     (pitch) and X-Z (roll) planes using the Lagrangian method.
% 2.  Includes all external forces (motor torques) and dissipative forces
%     (viscous friction) in the symbolic derivation.
% 3.  Symbolically solves the resulting system of equations for the angular
%     accelerations (ddot_theta and ddot_phi) for each plane.
% 4.  Uses the `matlabFunction` command to automatically generate separate,
%     optimized, and error-free numerical .m files for each acceleration.
%
% This corrected version performs a full derivation for BOTH planes and uses
% a more robust substitution method to avoid errors.
% -------------------------------------------------------------------------

% Sopra Vanno aggiunti momenti e forze esterne: il torque entra
% direttamente con -tau_ext_y, Per le forze ipotizziamo esse vengano
% applicate esattamente al centro della superficie superiore
% dell'upperbody, così che il braccio sia la metà dell'altezza
% dell'upperbody, e dunque il momento risultante sia: -(F_ext_z^2 +
% F_ext_p^2) x h/2. Problema che queste sono funzioni scalari, quindi
% bisognerebbe calcolare fuori il momento risultante e poi passarlo
% direttamente qui come -tau_F_ext_y. Stesso ragionamento vale per il
% frontal plane. Per lo Yaw, se ipotizziamo forze direttamente al centro
% della superficie allora le forze applicate non producono momento che
% varia l'angolazione theta_z ma dobbiamo integrare nell'equazione solo
% eventuali momenti esterni applicati all'asse z

clear; clc; 
close all;

fprintf('--- Starting Symbolic Derivation and Function Generation ---\n\n');

%% --- Define All Symbolic Variables ---
syms t
% Generalized coordinates for both planes (as functions of time for differentiation)
syms theta_y(t) phi_y(t) theta_x(t) phi_x(t)

% Derivatives, torques, and friction coefficients (as scalar symbols)
syms dot_theta_y dot_phi_y ddot_theta_y ddot_phi_y
syms dot_theta_x dot_phi_x ddot_theta_x ddot_phi_x
syms tau_y tau_x b_theta b_phi

% System Parameters
syms m_U I_Uy I_Ux l_U m_S I_S r_S h_app g

% --- ADDITION: External Wrench from Manipulator ---
syms F_px F_py F_pz      % External forces along body axes 
syms tau_px tau_py tau_pz     % External torques about body axes (roll, pitch)

% Create simple symbolic variables to represent the states for the final functions
syms th_y ph_y th_x ph_x

%% --- X-Z PLANE (PITCH) DYNAMICS ---

fprintf('1. Deriving equations for the Y-Z (Pitch) plane...\n');

% Kinetic Energy (T_y)
T_Sy = 0.5 * m_S * (r_S * diff(phi_y,t))^2 + 0.5 * I_S * diff(phi_y,t)^2;
v_Uy_x = r_S * diff(phi_y,t) + l_U * cos(theta_y) * diff(theta_y,t);
v_Uy_z = -l_U * sin(theta_y) * diff(theta_y,t);
T_Uy = 0.5 * m_U * (v_Uy_x^2 + v_Uy_z^2) + 0.5 * I_Uy * diff(theta_y,t)^2;
T_y = T_Sy + T_Uy;

% Potential Energy (V_y)
V_y = m_U * g * l_U * cos(theta_y);

% Lagrangian (L_y)
L_y = T_y - V_y;

% Euler-Lagrange Equations for Y-Z Plane
% Eq1 for theta_y
dLd_dot_theta_y = diff(L_y, diff(theta_y,t));
ddt_dLd_dot_theta_y = diff(dLd_dot_theta_y, t);
dLd_theta_y = diff(L_y, theta_y);
Q_theta_y = -tau_y -b_theta*diff(theta_y,t) + h_app*(F_px*cos(theta_y)-F_pz*sin(theta_y)) + tau_py; % Reaction torque and friction: 

EoM1_y = ddt_dLd_dot_theta_y - dLd_theta_y == Q_theta_y;

% Eq2 for phi_y
dLd_dot_phi_y = diff(L_y, diff(phi_y,t));
ddt_dLd_dot_phi_y = diff(dLd_dot_phi_y, t);
dLd_phi_y = diff(L_y, phi_y);
Q_phi_y = tau_y - b_phi * diff(phi_y,t) + r_S *  F_px; % Motor torque and friction
EoM2_y = ddt_dLd_dot_phi_y - dLd_phi_y == Q_phi_y;

% Substitute time-dependent variables with simple scalars BEFORE solving
% **CORRECTION**: Substitute highest-order derivatives first!
vars_y = [diff(theta_y,t,t), diff(phi_y,t,t), diff(theta_y,t), diff(phi_y,t), theta_y(t), phi_y(t)];
subs_vars_y = [ddot_theta_y, ddot_phi_y, dot_theta_y, dot_phi_y, th_y, ph_y];
EoM1_y_sub = subs(EoM1_y, vars_y, subs_vars_y);
EoM2_y_sub = subs(EoM2_y, vars_y, subs_vars_y);

% Solve for Y-Z accelerations
sol_y = solve([EoM1_y_sub, EoM2_y_sub], [ddot_theta_y, ddot_phi_y]);
ddot_theta_y_expr = sol_y.ddot_theta_y;
ddot_phi_y_expr = sol_y.ddot_phi_y;

fprintf('   ...Done.\n');

%% --- Y-Z PLANE (ROLL) DYNAMICS ---

fprintf('2. Deriving equations for the X-Z (Roll) plane...\n');

% Kinetic Energy (T_x) - Note the use of I_Ux
T_Sx = 0.5 * m_S * (r_S * diff(phi_x,t))^2 + 0.5 * I_S * diff(phi_x,t)^2;
v_Ux_y = -r_S * diff(phi_x,t) - l_U * cos(theta_x) * diff(theta_x,t);
v_Ux_z = -l_U * sin(theta_x) * diff(theta_x,t);
T_Ux = 0.5 * m_U * (v_Ux_y^2 + v_Ux_z^2) + 0.5 * I_Ux * diff(theta_x,t)^2;
T_x = T_Sx + T_Ux;

% Potential Energy (V_x)
V_x = m_U * g * l_U * cos(theta_x);

% Lagrangian (L_x)
L_x = T_x - V_x;

% Euler-Lagrange Equations for Y-Z Plane
% Eq1 for theta_x
dLd_dot_theta_x = diff(L_x, diff(theta_x,t));
ddt_dLd_dot_theta_x = diff(dLd_dot_theta_x, t);
dLd_theta_x = diff(L_x, theta_x);
Q_theta_x = -tau_x -b_theta*diff(theta_x,t) + h_app*(-F_py*cos(theta_x)-F_pz*sin(theta_x)) + tau_px;
EoM1_x = ddt_dLd_dot_theta_x - dLd_theta_x == Q_theta_x;

% Eq2 for phi_x
dLd_dot_phi_x = diff(L_x, diff(phi_x,t));
ddt_dLd_dot_phi_x = diff(dLd_dot_phi_x, t);
dLd_phi_x = diff(L_x, phi_x);
Q_phi_x = tau_x - b_phi * diff(phi_x,t) -r_S * F_py;
EoM2_x = ddt_dLd_dot_phi_x - dLd_phi_x == Q_phi_x;

% Substitute time-dependent variables with simple scalars BEFORE solving
% **CORRECTION**: Substitute highest-order derivatives first!
vars_x = [diff(theta_x,t,t), diff(phi_x,t,t), diff(theta_x,t), diff(phi_x,t), theta_x(t), phi_x(t)];
subs_vars_x = [ddot_theta_x, ddot_phi_x, dot_theta_x, dot_phi_x, th_x, ph_x];
EoM1_x_sub = subs(EoM1_x, vars_x, subs_vars_x);
EoM2_x_sub = subs(EoM2_x, vars_x, subs_vars_x);

% Solve for X-Z accelerations
sol_x = solve([EoM1_x_sub, EoM2_x_sub], [ddot_theta_x, ddot_phi_x]);
ddot_theta_x_expr = sol_x.ddot_theta_x;
ddot_phi_x_expr = sol_x.ddot_phi_x;

fprintf('   ...Done.\n');

%% --- GENERATE NUMERICAL MATLAB FUNCTIONS ---

fprintf('3. Generating .m files from symbolic expressions...\n');

% Define the order of input arguments for the generated functions
% This order MUST be respected in the final simulation function.
state_vars_y = [th_y, dot_theta_y, dot_phi_y];
input_vars_y = [tau_y];
param_vars = [m_U, I_Uy, I_Ux, l_U, m_S, I_S, r_S, g, b_theta, b_phi, h_app];

state_vars_x = [th_x, dot_theta_x, dot_phi_x];
input_vars_x = [tau_x];

% --- FINAL CORRECTION: Add external forces to function inputs ---
external_vars_y = [F_px, F_pz, tau_py];
external_vars_x = [F_py, F_pz, tau_px];

% Generate function for ddot_theta_y
matlabFunction(ddot_theta_y_expr, ...
               'File', 'copy_autogen_ddot_theta_y', ...
               'Vars', {state_vars_y, input_vars_y, param_vars,external_vars_y}, ...
               'Outputs', {'ddot_theta_y'});

% Generate function for ddot_phi_y
matlabFunction(ddot_phi_y_expr, ...
               'File', 'copy_autogen_ddot_phi_y', ...
               'Vars', {state_vars_y, input_vars_y, param_vars,external_vars_y}, ...
               'Outputs', {'ddot_phi_y'});

% Generate function for ddot_theta_x
matlabFunction(ddot_theta_x_expr, ...
               'File', 'copy_autogen_ddot_theta_x', ...
               'Vars', {state_vars_x, input_vars_x, param_vars,external_vars_x}, ...
               'Outputs', {'ddot_theta_x'});
               
% Generate function for ddot_phi_x
matlabFunction(ddot_phi_x_expr, ...
               'File', 'copy_autogen_ddot_phi_x', ...
               'Vars', {state_vars_x, input_vars_x, param_vars,external_vars_x}, ...
               'Outputs', {'ddot_phi_x'});

fprintf('   ...Done.\n\n');
fprintf('SUCCESS: Four distinct and correct function files have been created.\n');
