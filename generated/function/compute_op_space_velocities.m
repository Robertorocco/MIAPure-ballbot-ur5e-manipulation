function [q_dot_command,error_pos] = compute_op_space_velocities(robotModel, p, T_desired, x_ballbot, q_manip_actual)
% This function performs the entire operational space control calculation.
% It is intended to be called as a single extrinsic function from Simulink.

    %% --- 1. Calculate Actual Robot Pose ---
    theta_x = x_ballbot(1); theta_y = x_ballbot(2); theta_z = x_ballbot(3);
    phi_x   = x_ballbot(4); phi_y   = x_ballbot(5);
    
    pos_z = p.r_S + (p.h_U - p.r_S) * cos(theta_x) * cos(theta_y);
    p_base_actual = [p.r_S * phi_y; -p.r_S * phi_x; pos_z];

    o_base_eul = [theta_z, theta_y, theta_x];
    R_world_base = eul2rotm(o_base_eul, 'ZYX');

    T_world_base_actual = [ R_world_base, p_base_actual; 0, 0, 0, 1 ];
    
    endEffectorName = 'robotiq_85_base_link';
    T_base_ee_actual = getTransform(robotModel, q_manip_actual, endEffectorName);
    T_world_ee_actual = T_world_base_actual * T_base_ee_actual;

    %% --- 2. Calculate 6D Spatial Error ---
    error_pos = T_desired(1:3, 4) - T_world_ee_actual(1:3, 4);
    
    R_error   = T_desired(1:3, 1:3) * T_world_ee_actual(1:3, 1:3)';
    axang_error = rotm2axang(R_error);
    error_ori = axang_error(1:3)' * axang_error(4);
    spatial_error = [error_pos; zeros(3,1)];

    %% --- 3. Implement Jacobian Control Law ---
    Kp_pos = 0.5; Kp_ori = 0.04;
    K_matrix = diag([Kp_pos, Kp_pos, Kp_pos, Kp_ori, Kp_ori, Kp_ori]);
    
    spatial_velocity_cmd_world = K_matrix * spatial_error;
    
    v_cmd_world = spatial_velocity_cmd_world(1:3);
    w_cmd_world = spatial_velocity_cmd_world(4:6);
    v_cmd_base = R_world_base' * v_cmd_world;
    w_cmd_base = R_world_base' * w_cmd_world;
    spatial_velocity_cmd_base = [v_cmd_base; w_cmd_base];

    J_manip = geometricJacobian(robotModel, q_manip_actual, endEffectorName);
    
     q_dot_command = pinv(J_manip) * spatial_velocity_cmd_base;
end