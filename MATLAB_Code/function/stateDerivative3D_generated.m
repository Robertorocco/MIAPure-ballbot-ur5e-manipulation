function dx = stateDerivative3D_generated(x, p, tau)
    % SW was assumed to not spin relative to the ground
    
    % Unpack state vector (10 elements)
    theta_x = x(1); dot_theta_x = x(6);
    theta_y = x(2); dot_theta_y = x(7);
    theta_z = x(3); dot_theta_z = x(8);
    % phi_x   = x(4); % Not needed for dynamics
    % phi_y   = x(5); % Not needed for dynamics
    dot_phi_x   = x(9);
    dot_phi_y   = x(10);
    
    % Unpack input torques
    tau_x = tau(1);
    tau_y = tau(2);
    tau_z = tau(3);

    % --- Prepare arguments for auto-generated functions ---
    % Argument list for parameters (order MUST match the generator script)
    params_arg = [p.m_U, p.I_Uy, p.I_Ux, p.l_U, p.m_S, p.I_S, p.r_S, p.g, p.b_theta, p.b_phi];

    % --- X-Z Plane (Roll) Dynamics ---
    states_arg_x = [theta_x, dot_theta_x, dot_phi_x];
    inputs_arg_x = [tau_x];
    ddot_theta_x = autogen_ddot_theta_x(states_arg_x, inputs_arg_x, params_arg);
    ddot_phi_x   = autogen_ddot_phi_x(states_arg_x, inputs_arg_x, params_arg);

    % --- Y-Z Plane (Pitch) Dynamics ---
    states_arg_y = [theta_y, dot_theta_y, dot_phi_y];
    inputs_arg_y = [tau_y];
    ddot_theta_y = autogen_ddot_theta_y(states_arg_y, inputs_arg_y, params_arg);
    ddot_phi_y   = autogen_ddot_phi_y(states_arg_y, inputs_arg_y, params_arg);
    
    % --- Transversal Plane Dynamics (Yaw) ---
    % Using the simple decoupled model from your original script
    ddot_theta_z = (tau_z - p.fz * dot_theta_z) / p.Iz;
    
    % --- Assemble the state derivative vector dx ---
    dx = [
        dot_theta_x;    % dx(1) = d(theta_x)/dt
        dot_theta_y;    % dx(2) = d(theta_y)/dt
        dot_theta_z;    % dx(3) = d(theta_z)/dt
        dot_phi_x;      % dx(4) = d(phi_x)/dt
        dot_phi_y;      % dx(5) = d(phi_y)/dt
        ddot_theta_x;   % dx(6) = d(dot_theta_x)/dt
        ddot_theta_y;   % dx(7) = d(dot_theta_y)/dt
        ddot_theta_z;   % dx(8) = d(dot_theta_z)/dt
        ddot_phi_x;     % dx(9) = d(dot_phi_x)/dt
        ddot_phi_y;     % dx(10)= d(dot_phi_y)/dt
    ];
end