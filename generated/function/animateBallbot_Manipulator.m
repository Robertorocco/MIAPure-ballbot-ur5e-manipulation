%% ANIMATION FUNCTION
function animateBallbot_Manipulator(t, x, q, p, map_size, map_scale, waypoints, obstacles_def)
    % Generates a 3D animation of the ball-bot simulation results.
    
    % Setup the Figure and Axes
    figure('Name', '3D Ball-Bot Animation', 'NumberTitle', 'off', 'Color', 'w');
    ax = axes('DataAspectRatio',[1 1 1], 'Projection','perspective');
    view(35, 25); hold on; grid on; box on;
    xlabel('X Axis (m)'); ylabel('Y Axis (m)'); zlabel('Z Axis (m)');
    % Adjust Z-axis limit to accommodate obstacle height
    obstacle_height = 0.5 * map_scale; % Define a reasonable height for obstacles
    margin = 0.4 * map_scale;
    axis([0-margin-0.5 map_size(1)+margin 0-margin map_size(2)+margin 0 obstacle_height + 1.0]);
    light('Position',[2 2 3],'Style','infinite'); lighting gouraud;
    
    %% --- Plot 3D Obstacles and 2D Inflation Zones ---
    % Calculate the inflation radius
    inflation_radius = p.r_S + 0.15 * map_scale;
    
    % Loop through each obstacle definition to draw it
    for i = 1:size(obstacles_def, 1)
        obs_rect = obstacles_def(i, :); % [x, y, width, height]
        
        % 1. Draw the 2D inflated area (flat on the ground)
        plot_rounded_rectangle_2d(obs_rect, inflation_radius, [1 0.8 0.8], 0.5);

        % 2. Draw the original 3D obstacle on top
        x_obs = obs_rect(1); y_obs = obs_rect(2); w_obs = obs_rect(3); h_obs = obs_rect(4);
        
        % Vertices for a rectangular prism (cuboid)
        verts = [
            x_obs,      y_obs,      0;
            x_obs+w_obs, y_obs,      0;
            x_obs+w_obs, y_obs+h_obs, 0;
            x_obs,      y_obs+h_obs, 0;
            x_obs,      y_obs,      obstacle_height;
            x_obs+w_obs, y_obs,      obstacle_height;
            x_obs+w_obs, y_obs+h_obs, obstacle_height;
            x_obs,      y_obs+h_obs, obstacle_height;
        ];
        
        % Faces for a rectangular prism
        faces = [
            1 2 3 4; % Bottom face
            5 6 7 8; % Top face
            1 2 6 5; % Side face 1
            2 3 7 6; % Side face 2
            3 4 8 7; % Side face 3
            4 1 5 8; % Side face 4
        ];
        
        patch('Vertices', verts, 'Faces', faces, 'FaceColor', [0.6 0.2 0.2], 'EdgeColor', 'k', 'FaceLighting', 'gouraud');
    end

    % (The rest of your function, including robot model, paths, and animation loop, remains unchanged)
    % ...

    % Create the Graphical Model
    robot_transform = hgtransform('Parent', ax);
    % 1. Spherical Wheel
    [sx, sy, sz] = sphere(40);
    surf(p.r_S*sx, p.r_S*sy, p.r_S*sz, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'Parent', robot_transform, 'FaceLighting','gouraud','AmbientStrength',0.4);
    
    % 2. Main Body Components...
    motor_radius = 0.05; motor_height = 0.07; omni_radius = p.r_ow;
    omni_width = 0.04; chassis_dist = p.r_ow+p.r_S; angles = [0, 120, 240];
    for i = 1:3
        assembly_angle = angles(i) * pi/180;
        assembly_transform = hgtransform('Parent', robot_transform, 'Matrix', makehgtform('zrotate', assembly_angle));
        [cx,cy,cz] = cylinder(motor_radius, 20);
        motor = surf(cx, cy, cz * motor_height, 'FaceColor', [0.5 0.5 0.6], 'EdgeColor', 'none');
        T_motor = makehgtform('translate', [chassis_dist*cos(pi/4), 0, chassis_dist*sin(pi/4)]);
        set(motor, 'Parent', hgtransform('Parent', assembly_transform, 'Matrix', T_motor))
        [ox,oy,oz] = cylinder(omni_radius, 20);
        omni = surf(ox, oy, (oz - 0.5) * omni_width, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
        T_omni_translate = makehgtform('translate', [chassis_dist*cos(pi/4), 0, chassis_dist*sin(pi/4)]);
        T_omni_tilt = makehgtform('yrotate', -pi/4);
        set(omni, 'Parent', hgtransform('Parent', assembly_transform, 'Matrix', T_omni_translate * T_omni_tilt));
    end
    
    % 3. Top Hexagonal Frame...
    hex_radius = 0.18; hex_height = 0.20; hex_thick = 0.02; hex_verts = zeros(12, 3);
    for i = 1:6
        angle = (i-1)*60*pi/180;
        hex_verts(i, :) = [hex_radius*cos(angle), hex_radius*sin(angle), hex_height];
        hex_verts(i+6, :) = [hex_radius*cos(angle), hex_radius*sin(angle), hex_height+hex_thick];
    end
    hex_faces = [1 2 8; 1 8 7; 2 3 9; 2 9 8; 3 4 10; 3 10 9; 4 5 11; 4 11 10; 5 6 12; 5 12 11; 6 1 7; 6 7 12; 1 2 3; 1 3 4; 1 4 5; 1 5 6; 7 8 9; 7 9 10; 7 10 11; 7 11 12];
    patch('Vertices', hex_verts, 'Faces', hex_faces, 'FaceColor', [0.7 0.7 0.75], 'Parent', robot_transform);
    
    % %%ADDITION
    % 4. Add UR5e manipulator on top
    meshPath = fullfile(pwd, 'ros_packages', 'meshes');
    ur5e = importrobot("UR5e.urdf", 'MeshPath', {meshPath});
    ur5e.DataFormat = 'row';  ur5e.Gravity = [0 0 -9.81];   
    ur5e.removeBody('base');
    ur5e.removeBody('ground_plane');
    % New tree
    ur5e_mobile = rigidBodyTree('DataFormat','row');

    % Dummy base with floating joint
    dummyBase = rigidBody("mobile_base");
    jnt = rigidBodyJoint("float1","floating");
    dummyBase.Joint = jnt;

    addBody(ur5e_mobile, dummyBase, "base");
    addSubtree(ur5e_mobile, "mobile_base", ur5e); 

    % Create an hgtransform to carry the manipulator base
    mount_height = hex_height + hex_thick;
    pos = [1 0 0 0 0 0 mount_height];  
    q0 = [pos 0 -2.356 2.356 -1.57 -1.57 0];  % your desired start config
    show(ur5e_mobile, q0, 'Parent', ax, 'Frames','off','PreservePlot',false,'Visuals','on');
    % Hide manipulator graphics from the legend
    set(findobj(ax, 'Type', 'Patch'), 'HandleVisibility','off');
    set(findobj(ax, 'Type', 'Surface'), 'HandleVisibility','off');

    %%STOP ADDITION


    % Plot the full trajectory path on the ground
    pos_x_history = p.r_S * x(:,5);
    pos_y_history = -p.r_S * x(:,4);
    h_actual = plot3(pos_x_history, pos_y_history, zeros(size(pos_x_history)), 'g-', 'LineWidth', 1.5);
    h_ref = plot3(waypoints(:,1), waypoints(:,2), zeros(size(waypoints(:,1))), 'b--', 'LineWidth', 2);
    
    % --- Update Legend ---
    h_inf = patch(NaN, NaN, [1 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    h_obs = patch(NaN, NaN, NaN, 'FaceColor', [0.6 0.2 0.2], 'EdgeColor', 'k');
    legend([h_actual, h_ref, h_obs, h_inf], ...
           'Actual Path', 'Reference Path', 'Obstacle', 'Inflation Zone', 'Location', 'northwest');
    
    % Animation Loop
    disp('Starting animation...');

            % --- Camera setup ---
    view(ax, [179.0653 22.5028]);
    axis(ax, 'vis3d');
    camproj(ax, 'perspective');
    campos(ax, [3.0830   38.2411   15.4336]);
    camtarget(ax, [2.5000    2.5000    0.6250]);
    camva(ax, 'manual');
    camup(ax, [0 0 1]);
    
    % --- Video recording setup ---
    v = VideoWriter('ballbot_animation.mp4', 'MPEG-4');
    v.FrameRate = 30;
    open(v);
    step_ini=250;
    for k = 1:step_ini:120000
        theta_x_k = x(k, 1); theta_y_k = x(k, 2); theta_z_k = x(k, 3);
        quat = eul2quat([theta_x_k, theta_y_k, theta_z_k], 'XYZ'); 
        phi_x_k = x(k, 4); phi_y_k = x(k, 5);
       
        pos_x = p.r_S * phi_y_k;
        pos_y = -p.r_S * phi_x_k;
        pos_z = p.r_S;
        
        q_k = [quat, pos_x, pos_y, pos_z+mount_height, q(k,:)];

        T_translate = makehgtform('translate', [pos_x, pos_y, pos_z]);
        T_rot_z = makehgtform('zrotate', theta_z_k);
        T_rot_y = makehgtform('yrotate', theta_y_k);
        T_rot_x = makehgtform('xrotate', theta_x_k);

        robot_transform.Matrix = T_translate * T_rot_z * T_rot_y * T_rot_x;

        show(ur5e_mobile, q_k, 'Parent', ax, 'Frames','off','PreservePlot',false,'Visuals','on');
        % Hide manipulator graphics from the legend
        set(findobj(ax, 'Type', 'Patch'), 'HandleVisibility','off');
        set(findobj(ax, 'Type', 'Surface'), 'HandleVisibility','off');
        set(findobj(ax, 'Type', 'Line'), 'HandleVisibility','off');
        %set(findall(ax, '-property', 'HandleVisibility'), 'HandleVisibility', 'off');


        title(ax, sprintf('Ball-Bot Simulation | Time: %.2f s', t(k)));
        drawnow;
        frame = getframe(gcf);
     writeVideo(v, frame);
     
        %pause(0.01);
    end
    step_end=500;
    for k = 120000:step_end:length(t)
        theta_x_k = x(k, 1); theta_y_k = x(k, 2); theta_z_k = x(k, 3);
        quat = eul2quat([theta_x_k, theta_y_k, theta_z_k], 'XYZ'); 
        phi_x_k = x(k, 4); phi_y_k = x(k, 5);
       
        pos_x = p.r_S * phi_y_k;
        pos_y = -p.r_S * phi_x_k;
        pos_z = p.r_S;
        
        q_k = [quat, pos_x, pos_y, pos_z+mount_height, q(k,:)];

        T_translate = makehgtform('translate', [pos_x, pos_y, pos_z]);
        T_rot_z = makehgtform('zrotate', theta_z_k);
        T_rot_y = makehgtform('yrotate', theta_y_k);
        T_rot_x = makehgtform('xrotate', theta_x_k);

        robot_transform.Matrix = T_translate * T_rot_z * T_rot_y * T_rot_x;

        show(ur5e_mobile, q_k, 'Parent', ax, 'Frames','off','PreservePlot',false,'Visuals','on');
        % Hide manipulator graphics from the legend
        set(findobj(ax, 'Type', 'Patch'), 'HandleVisibility','off');
        set(findobj(ax, 'Type', 'Surface'), 'HandleVisibility','off');
        set(findobj(ax, 'Type', 'Line'), 'HandleVisibility','off');
        %set(findall(ax, '-property', 'HandleVisibility'), 'HandleVisibility', 'off');


        title(ax, sprintf('Ball-Bot Simulation | Time: %.2f s', t(k)));
        drawnow;
        frame = getframe(gcf);
         writeVideo(v, frame);
        %pause(0.01);
    end
    close(v);
    disp('Video saved as ballbot_animation.mp4');
    disp('Animation complete.');
end

function plot_rounded_rectangle_2d(rect, r, face_color, face_alpha)
    % Plots a 2D rounded rectangle on the z=0 plane.
    % rect: [x, y, w, h] of the base rectangle
    % r: radius of the corners
    
    x = rect(1); y = rect(2); w = rect(3); h = rect(4);
    
    % Define angles for a smooth quarter circle
    t = linspace(0, pi/2, 25);
    
    % Generate points for each corner arc
    x_tr = (x + w) + r * cos(t);
    y_tr = (y + h) + r * sin(t);
    
    x_tl = x + r * cos(t + pi/2);
    y_tl = (y + h) + r * sin(t + pi/2);
    
    x_bl = x + r * cos(t + pi);
    y_bl = y + r * sin(t + pi);
    
    x_br = (x + w) + r * cos(t + 3*pi/2);
    y_br = y + r * sin(t + 3*pi/2);
    
    % Combine all vertices into a single polygon
    verts_x = [x_tr, x_tl, x_bl, x_br];
    verts_y = [y_tr, y_tl, y_bl, y_br];
    
    % Plot the filled shape on the ground (z=0)
    patch(verts_x, verts_y, zeros(size(verts_x)), face_color, ...
          'FaceAlpha', face_alpha, 'EdgeColor', 'none');
end