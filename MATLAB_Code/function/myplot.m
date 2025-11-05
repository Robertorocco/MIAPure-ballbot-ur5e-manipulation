function myplot(t, x, p, tau_history, tau_history_ow)
% MYPLOT Generates specific plots for the ballbot simulation results.
%
% This version is GENERIC. It accepts a pre-calculated history of the
% input torques, making it independent of the control strategy used.
%
% INPUTS:
%   t           - Time vector from the simulation
%   x           - State matrix from the simulation
%   p           - Parameters struct
%   tau_history - Matrix (Nx3) of the input torques [tau_x, tau_y, tau_z]
%                 applied at each time step.
%   tau_history_ow - Matrix (Nx3) of the omni-wheel torques.
% =========================================================================
% PART 1: EXTRACT AND PROCESS DATA
% =========================================================================
fprintf('Generating plots...\n');
% --- Extract States ---
theta_x = x(:,1);
theta_y = x(:,2);
theta_z = x(:,3);
phi_x   = x(:,4);
phi_y   = x(:,5);
dot_theta_x = x(:,6);
dot_theta_y = x(:,7);
dot_theta_z = x(:,8);
dot_phi_x   = x(:,9);
dot_phi_y   = x(:,10);
% --- Recalculate Accelerations (REMOVED as requested) ---
% --- Calculate Cartesian Path ---
pos_x = p.r_S * phi_y;
pos_y = -p.r_S * phi_x; % Negative sign for standard coordinate frames
% =========================================================================
% PART 2: GENERATE PLOTS
% =========================================================================
% --- Figure 1: Upper Body Data ---
figure('Name', 'Upper Body Data', 'NumberTitle', 'off');
sgtitle('Upper Body State Evolution');
% Subplot 1: Tilt Angles
subplot(2,1,1); % Changed from 3,1,1
plot(t, rad2deg(theta_x), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(theta_y), 'b-', 'LineWidth', 1.5);
hold off;
grid on;
title('Tilt Angles (Unwrapped)');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('\theta_x (Roll)', '\theta_y (Pitch)', 'Interpreter', 'tex', 'Location', 'best');
% Subplot 2: Tilt Angular Speeds
subplot(2,1,2); % Changed from 3,1,2
plot(t, dot_theta_x, 'r-', 'LineWidth', 1.5);
hold on;
plot(t, dot_theta_y, 'b-', 'LineWidth', 1.5);
hold off;
grid on;
title('Tilt Angular Speeds');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('d\theta_x/dt', 'd\theta_y/dt', 'Interpreter', 'tex', 'Location', 'best');
% Subplot 3: Tilt Angular Accelerations (REMOVED)
% --- Figure 2: Spherical Wheel Data ---
figure('Name', 'Spherical Wheel Data', 'NumberTitle', 'off');
sgtitle('Spherical Wheel State Evolution');
% Subplot 1: Cartesian Path
subplot(2,1,1); % Changed from 3,1,1
plot(pos_x, pos_y, 'k-', 'LineWidth', 1.5);
hold on;
plot(pos_x(1), pos_y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % Start
plot(pos_x(end), pos_y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % End
hold off;
grid on;
axis equal;
title('Path in XY Plane');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Path', 'Start', 'End', 'Location', 'best');
% Subplot 2: Angular Speeds
subplot(2,1,2); % Changed from 3,1,2
plot(t, dot_phi_x, 'r-', 'LineWidth', 1.5);
hold on;
plot(t, dot_phi_y, 'b-', 'LineWidth', 1.5);
hold off;
grid on;
title('Angular Speeds');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('d\phi_x/dt (Roll-inducing)', 'd\phi_y/dt (Pitch-inducing)', 'Interpreter', 'tex', 'Location', 'best');
% Subplot 3: Angular Accelerations (REMOVED)
% --- Figure 3: Yaw Plane Data ---
figure('Name', 'Yaw Plane Data', 'NumberTitle', 'off');
sgtitle('Yaw State Evolution');
% Subplot 1: Yaw Angle
subplot(2,1,1);
plot(t, rad2deg(theta_z), 'b-', 'LineWidth', 1.5);
grid on;
title('Yaw Angle (Unwrapped)');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('\theta_z (Yaw)', 'Interpreter', 'tex', 'Location', 'best');
% Subplot 2: Yaw Angular Speed
subplot(2,1,2);
plot(t, dot_theta_z, 'b-', 'LineWidth', 1.5);
grid on;
title('Yaw Angular Speed');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('d\theta_z/dt', 'Interpreter', 'tex', 'Location', 'best');
% --- Figure 4: Control Input Torques ---
figure('Name', 'Control Input Torques', 'NumberTitle', 'off');
sgtitle('Applied Control Torques (SW)');
plot(t, tau_history(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, tau_history(:,2), 'g-', 'LineWidth', 1.5);
plot(t, tau_history(:,3), 'b-', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('\tau_x (Roll)', '\tau_y (Pitch)', '\tau_z (Yaw)', 'Interpreter', 'tex', 'Location', 'best');
% --- Figure 5: Control Input Torques applied by OW ---
figure('Name', 'Control Input Torques (OW)', 'NumberTitle', 'off');
sgtitle('Applied Control Torques (OW)');
plot(t, tau_history_ow(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, tau_history_ow(:,2), 'g-', 'LineWidth', 1.5);
plot(t, tau_history_ow(:,3), 'b-', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('\tau_1 (Roll)', '\tau_2 (Pitch)', '\tau_3 (Yaw)', 'Interpreter', 'tex', 'Location', 'best');
fprintf('Plots generated successfully.\n');
end