function animateBallbot(t, x, p)
% ANIMATEBALLBOT Creates a 3D animation of the ballbot simulation results.
%
% INPUTS:
%   t - Time vector from the simulation
%   x - State matrix from the simulation
%   p - Parameters struct

% =========================================================================
% 3D ANIMATION
% =========================================================================
fprintf('\nStarting 3D animation...\n');

% --- Setup the Figure and Axes ---
figure('Name', '3D Ball-Bot Animation', 'NumberTitle', 'off', 'Color', 'w');
ax = axes('DataAspectRatio',[1 1 1], 'Projection','perspective');
view(35, 25);
hold on; grid on; box on;
xlabel('X Axis (m)');
ylabel('Y Axis (m)');
zlabel('Z Axis (m)');
axis([-1 1 -1 1 0 1.2]); % Set axis limits
light('Position',[2 2 3],'Style','infinite');
lighting gouraud;

% --- Create the Graphical Model ---
% A single transform for the entire robot assembly.
robot_transform = hgtransform('Parent', ax);

% 1. Spherical Wheel
[sx, sy, sz] = sphere(40);
surf(p.r_S*sx, p.r_S*sy, p.r_S*sz, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'Parent', robot_transform, 'FaceLighting','gouraud','AmbientStrength',0.4);

% 2. Main Body Components (approximated from image)
motor_radius = 0.05;
motor_height = 0.07;
omni_radius = 0.0625;
omni_width = 0.04;
chassis_dist = 0.14;
angles = [90, 210, 330];
for i = 1:3
    assembly_angle = angles(i) * pi/180;
    assembly_transform = hgtransform('Parent', robot_transform, 'Matrix', makehgtform('zrotate', assembly_angle));
    
    % Motor
    [cx,cy,cz] = cylinder(motor_radius, 20);
    motor = surf(cx, cy, cz * motor_height, 'FaceColor', [0.5 0.5 0.6], 'EdgeColor', 'none');
    T_motor = makehgtform('translate', [chassis_dist, 0, omni_radius + 0.01]);
    set(motor, 'Parent', hgtransform('Parent', assembly_transform, 'Matrix', T_motor));
    
    % Omni-Wheel
    [ox,oy,oz] = cylinder(omni_radius, 20);
    omni = surf(ox, oy, (oz - 0.5) * omni_width, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
    T_omni_translate = makehgtform('translate', [chassis_dist, 0, 0]);
    T_omni_tilt = makehgtform('yrotate', -pi/4);
    set(omni, 'Parent', hgtransform('Parent', assembly_transform, 'Matrix', T_omni_translate * T_omni_tilt));
end

% 3. Top Hexagonal Frame
hex_radius = 0.18;
hex_height = 0.35;
hex_thick = 0.02;
hex_verts = zeros(12, 3);
for i = 1:6
    angle = (i-1)*60*pi/180;
    hex_verts(i, :) = [hex_radius*cos(angle), hex_radius*sin(angle), hex_height];
    hex_verts(i+6, :) = [hex_radius*cos(angle), hex_radius*sin(angle), hex_height+hex_thick];
end
% **CORRECTED** face definition: All faces are defined as triangles
% to ensure the face matrix has a consistent number of columns.
hex_faces = [
    1 2 8; 1 8 7;   % Side face 1
    2 3 9; 2 9 8;   % Side face 2
    3 4 10; 3 10 9; % Side face 3
    4 5 11; 4 11 10;% Side face 4
    5 6 12; 5 12 11;% Side face 5
    6 1 7; 6 7 12;  % Side face 6
    1 2 3; 1 3 4; 1 4 5; 1 5 6;      % Bottom face (triangulated)
    7 8 9; 7 9 10; 7 10 11; 7 11 12  % Top face (triangulated)
];
patch('Vertices', hex_verts, 'Faces', hex_faces, 'FaceColor', [0.7 0.7 0.75], 'Parent', robot_transform);

% --- Animation Loop ---
% Use a smaller number of frames for faster animation
num_frames = 200;
time_vector = linspace(t(1), t(end), num_frames);
state_vector = interp1(t, x, time_vector);

for k = 1:num_frames
    % Extract states at the interpolated time step
    theta_x_k = state_vector(k, 1); % Roll
    theta_y_k = state_vector(k, 2); % Pitch
    theta_z_k = state_vector(k, 3); % Yaw
    phi_x_k   = state_vector(k, 4);
    phi_y_k   = state_vector(k, 5);


    % 1. Calculate the sphere center's 3D position
    pos_x = p.r_S * phi_y_k;
    pos_y = -p.r_S * phi_x_k; % Corrected sign for standard coordinate frames
    pos_z = p.r_S;
    
    % 2. Create the complete transformation matrix
    T_translate = makehgtform('translate', [pos_x, pos_y, pos_z]);
    T_rot_z = makehgtform('zrotate', theta_z_k);
    T_rot_y = makehgtform('yrotate', theta_y_k);
    T_rot_x = makehgtform('xrotate', theta_x_k);
    
    % 3. Apply the combined matrix to the parent transform
    robot_transform.Matrix = T_translate * T_rot_z * T_rot_y * T_rot_x;
    
    title(ax, sprintf('Ball-Bot Simulation | Time: %.2f s', time_vector(k)));
    drawnow;
    pause(0.05);
end

disp('Animation complete.');
end
