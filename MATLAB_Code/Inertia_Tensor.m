%% --- 1. Load Robot and Set Properties ---
clear; clc;

% Path to your URDF (adjust if it's in another folder)
try
    robot = importrobot("UR5e.urdf");
catch
    error("UR5e.urdf not found. Please ensure the file is in the current path.");
end

robot.Gravity = [0, 0, -9.81];
robot.DataFormat = "column";

% Define a sample joint configuration for the calculation
config = [0; -2.356; 2.356; -1.57; -1.57; 0];


%% --- 2. Calculate Properties in the Robot's Original Base Frame ---
[total_mass, com_in_base, inertia_tensor_com] = getCompositeInertia(robot, config);


%% --- 3. Transform Results to the New Reference Frame ---
z_offset = 0.5; % 33.55 cm
p_base_in_new = [0; 0; z_offset];
com_in_new_frame = p_base_in_new + com_in_base;


%% --- 4. Calculate Inertia Tensor about the New Frame's Origin ---
% Use the Parallel Axis Theorem to shift the inertia tensor from the COM
% to the origin of the new reference frame.
d = com_in_new_frame; % Vector from new origin to COM
I_about_new_origin = inertia_tensor_com + total_mass * ( (d'*d)*eye(3) - (d*d') );


%% --- 5. Display Final Results ---
fprintf('--- Calculation Results ---\n\n');
fprintf('Total Mass: %.4f kg\n\n', total_mass);

fprintf('Center of Mass relative to NEW FRAME:\n');
fprintf('  X: %.4f m\n  Y: %.4f m\n  Z: %.4f m\n\n', com_in_new_frame(1), com_in_new_frame(2), com_in_new_frame(3));

disp('Inertia Tensor about the Composite COM:');
disp(inertia_tensor_com);

fprintf('\n'); % Add a space for clarity

disp('Inertia Tensor about the NEW FRAME ORIGIN:');
disp(I_about_new_origin);


%% --- Supporting Function (Unchanged) ---
function [total_mass, com_position, inertia_tensor_com] = getCompositeInertia(robot, config)
    % GETCOMPOSITEINERTIA Calculates the composite dynamic properties of a rigidBodyTree.
    % This function correctly calculates properties relative to the base of the 'robot' object passed to it.
    
    if ~strcmp(robot.DataFormat, 'column')
        error('This function requires the robot.DataFormat to be set to "column".');
    end

    total_mass = 0;
    first_moment_of_mass = zeros(3, 1);
    I_total_about_origin = zeros(3, 3);

    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        m_i = body.Mass;
        if m_i == 0
            continue;
        end
        
        T_base_to_link = getTransform(robot, config, body.Name);
        R_base_to_link = T_base_to_link(1:3, 1:3);
        p_link_origin_in_base = T_base_to_link(1:3, 4);
        
        com_in_link_frame = body.CenterOfMass';
        
        inertia_vec = body.Inertia;
        Ixx = inertia_vec(1); Iyy = inertia_vec(2); Izz = inertia_vec(3);
        Iyz = inertia_vec(4); Ixz = inertia_vec(5); Ixy = inertia_vec(6);
        I_in_link_frame = [ Ixx, -Ixy, -Ixz;
                           -Ixy,  Iyy, -Iyz;
                           -Ixz, -Iyz,  Izz ];
        
        p_com_in_base = p_link_origin_in_base + R_base_to_link * com_in_link_frame;
        I_com_in_base_orientation = R_base_to_link * I_in_link_frame * R_base_to_link';
        
        total_mass = total_mass + m_i;
        first_moment_of_mass = first_moment_of_mass + m_i * p_com_in_base;
        
        d_link = p_com_in_base;
        I_link_about_origin = I_com_in_base_orientation + m_i * ( (d_link'*d_link)*eye(3) - (d_link*d_link') );
        I_total_about_origin = I_total_about_origin + I_link_about_origin;
    end
    
    if total_mass > 1e-10
        com_position = first_moment_of_mass / total_mass;
        d_total = com_position;
        inertia_tensor_com = I_total_about_origin - total_mass * ( (d_total'*d_total)*eye(3) - (d_total*d_total') );
    else
        com_position = zeros(3,1);
        inertia_tensor_com = zeros(3,3);
    end
end