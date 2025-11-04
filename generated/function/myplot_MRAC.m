function myplot_MRAC(t, x, r, p, tau_history_ow, gains_hystory)
% MYPLOT Generates specific plots for the ballbot simulation results.
%
% This version is GENERIC. It accepts a pre-calculated history of the
% input torques, making it independent of the control strategy used.
%
% INPUTS:
%   t           - Time vector from the simulation
%   x           - State matrix from the simulation
%   r           - Reference Input
%   p           - Parameters struct
%   tau_history_ow - Matrix (Nx3) of the omni-wheel torques.
% =========================================================================
% PART 1: EXTRACT AND PROCESS DATA
% =========================================================================
fprintf('Generating plots...\n');
% --- Extract States ---
theta_x = x(:,1);
theta_y = x(:,5);
theta_z = x(:,9);
phi_x   = x(:,3);
phi_y   = x(:,7);
dot_theta_x = x(:,2);
dot_theta_y = x(:,6);
dot_theta_z = x(:,10);
dot_phi_x   = x(:,4);
dot_phi_y   = x(:,8);

% --- Calculate Cartesian Path ---
pos_x = p.r_S * phi_y;
pos_y = -p.r_S * phi_x;
pos_xd = r(:,1);
pos_yd = r(:,2); 

figure('Name', 'MRAC NAVIGATION SIMULATION', 'color', 'w');
sgtitle('Ballbot MRAC Simulation Results', 'FontSize', 14);

% -------------------------------------------------------------------------
% Plot 1: Upper Body Tilt Angles (Top-Left)
% -------------------------------------------------------------------------
ax1 = subplot(2, 2, 1);
plot(ax1, t, rad2deg(theta_x), 'r-', 'LineWidth', 2);
hold(ax1, 'on');
plot(ax1, t, rad2deg(theta_y), 'b-', 'LineWidth', 2);
plot(ax1, t, rad2deg(theta_z), 'g-', 'LineWidth', 2);
hold(ax1, 'off');
title(ax1, 'Upper Body Tilt Angles');
xlabel(ax1, 'Time (s)');
ylabel(ax1, 'Angle (degrees)');
legend(ax1, '$\theta_x$ (Roll)', '$\theta_y$ (Pitch)',  '$\theta_z$ (Yaw)', 'Interpreter', 'latex', 'Location', 'northeast');
grid(ax1, 'on');
box(ax1, 'on');
axis(ax1, 'tight');

% -------------------------------------------------------------------------
% Plot 2: Spherical Wheel Angular Speed (Top-Right)
% -------------------------------------------------------------------------
ax2 = subplot(2, 2, 2);
plot(ax2, t, dot_phi_x, 'r-', 'LineWidth', 2);
hold(ax2, 'on');
plot(ax2, t, dot_phi_y, 'b-', 'LineWidth', 2);
hold(ax2, 'off');
title(ax2, 'Spherical Wheel Angular Speed');
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Speed (rad/s)');
legend(ax2, '$\dot{\phi}_x$', '$\dot{\phi}_y$', 'Interpreter', 'latex', 'Location', 'northeast');
grid(ax2, 'on');
box(ax2, 'on');
axis(ax2, 'tight'); %% --- ADDED to fit data perfectly --- %%

% -------------------------------------------------------------------------
% Plot 3: Upper Body Tilt Velocity (Bottom-Left)
% -------------------------------------------------------------------------
ax3 = subplot(2, 2, 3);
plot(ax3, t, dot_theta_x, 'r-', 'LineWidth', 2);
hold(ax3, 'on');
plot(ax3, t, dot_theta_y, 'b-', 'LineWidth', 2);
hold(ax3, 'off');
title(ax3, 'Upper Body Tilt Velocity');
xlabel(ax3, 'Time (s)');
ylabel(ax3, 'Speed (rad/s)');
legend(ax3, '$\dot{\theta}_x$', '$\dot{\theta}_y$', 'Interpreter', 'latex', 'Location', 'southeast');
grid(ax3, 'on');
box(ax3, 'on');
axis(ax3, 'tight'); 

%% --- ADDED: Zoomed Inset Box for Subplot 3 (Corrected) --- %%
% 1. Define zoom range
t_zoom_start = 5;
t_zoom_end = 30;

% 2. Find the logical indices for the data within this time range
idx_zoom = (t >= t_zoom_start) & (t <= t_zoom_end);

% 3. Extract the subset of data *before* plotting
t_zoomed = t(idx_zoom);
dot_theta_x_zoomed = dot_theta_x(idx_zoom);
dot_theta_y_zoomed = dot_theta_y(idx_zoom);

% 4. Create a new axes for the inset
%    You may need to adjust [left bottom width height] for your screen
ax_inset = axes('Position', [0.25 0.25 0.2 0.1]); 

% 5. Plot ONLY the subset of data
plot(ax_inset, t_zoomed, dot_theta_x_zoomed, 'r-', 'LineWidth', 1.5);
hold(ax_inset, 'on'); grid(ax_inset, 'on');
plot(ax_inset, t_zoomed, dot_theta_y_zoomed, 'b-', 'LineWidth', 1.5);
hold(ax_inset, 'off');

% 6. Now 'axis tight' will only see the subset of data
axis(ax_inset, 'tight'); 
box(ax_inset, 'on');

% -------------------------------------------------------------------------
% Plot 4: Applied Omni-Wheel Torques (Bottom-Right)
% -------------------------------------------------------------------------
ax4 = subplot(2, 2, 4);
plot(ax4, t, tau_history_ow(:, 1), 'r-','LineWidth', 2);
hold(ax4, 'on');
plot(ax4, t, tau_history_ow(:, 2), 'g-','LineWidth', 2);
plot(ax4, t, tau_history_ow(:, 3), 'b-','LineWidth', 2);
hold(ax4, 'off');
% Calculate min and max peaks from the FULL dataset for an accurate title
min_peak = min(tau_history_ow, [], 'all');
max_peak = max(tau_history_ow, [], 'all');
% Update title to reflect that the view is zoomed
title_str = sprintf('Applied OW Torques (Zoomed)\nActual Peaks: [%.1f, %.1f] Nm', min_peak, max_peak);
title(ax4, title_str);
xlabel(ax4, 'Time (s)');
ylabel(ax4, 'Torque (Nm)');
legend(ax4, '$\tau_1$', '$\tau_2$', '$\tau_3$', 'Interpreter', 'latex', 'Location', 'southeast');
grid(ax4, 'on');
box(ax4, 'on');
ylim(ax4, [-5, 5]);
xlim(ax4, [t(1), t(end)]); 

% exportgraphics(gcf, 'C:\Users\chiar_zjcy1by\Desktop\Field and Service Robotics\FinalProjPlots\mrac_ballbot_results.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none')
% -------------------------------------------------------------------------
% Figure 2: Spherical Wheel Path (Unmodified)
% -------------------------------------------------------------------------
figure('Name', 'Spherical Wheel Path', 'NumberTitle', 'off');
plot(pos_x, pos_y, 'k-', 'LineWidth', 3);
hold on;
plot(pos_xd, pos_yd, 'r--', 'LineWidth', 3);
plot(pos_x(1), pos_y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % Start
plot(pos_x(end), pos_y(end), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8); % End
hold off;
grid on;
xlim([min(pos_x)+sign(min(pos_x))*0.2 max(pos_x)+sign(max(pos_x))*0.2]);
ylim([min(pos_y)+sign(min(pos_y))*0.2 max(pos_y)+sign(max(pos_y))*0.2]);
title('Path in XY Plane');
xlabel('X Position (m)');
ylabel('Y Position (m)');
%% --- orientation arrows along the actual path (theta_z) ---
N_arrows = 30;
nPts = length(t);
if length(pos_x) == nPts && length(pos_y) == nPts && length(theta_z) == nPts && N_arrows > 0

    % Pick indices regularly along the path
    idx = round(linspace(1, nPts, min(N_arrows, nPts)));

    % Direction unit vectors from theta_z (assuming theta_z in radians)
    ux = cos(theta_z(idx));
    uy = sin(theta_z(idx));
    arrow_len = 0.2; % fixed length
    % Final arrow vectors
    u = ux * arrow_len;
    v = uy * arrow_len;

    hold on;
    hq = quiver(pos_x(idx), pos_y(idx), u, v, 0, 'LineWidth', 1.2, 'MaxHeadSize', 1.2);
    set(hq, 'Color', [0 0.45 0.74]);
    plot(pos_x(idx), pos_y(idx), 'o', 'MarkerSize', 4, 'MarkerFaceColor', [0 0.45 0.74], 'MarkerEdgeColor','k','HandleVisibility','off');
    hold off;
else
    warning('Cannot plot orientation arrows: inconsistent lengths or missing theta_z/pos vectors.');
end
legend('Actual Path', 'Desired Path', 'Start', 'End', 'Robot Orientation', 'Location', 'southeast');

% exportgraphics(gcf, 'C:\Users\chiar_zjcy1by\Desktop\Field and Service Robotics\FinalProjPlots\mrac_ballbot_path.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none')
% -------------------------------------------------------------------------

% Figure 3: MRAC Gains (Thicker lines & Y-padding)
% -------------------------------------------------------------------------
planes = {'Frontal','Sagittal','Transverse'};   % columns
contribs = {'Feedforward','State feedback','Robustifying action'}; % rows
% Prepare a cell array G{row, col} to store data for plotting
G = cell(3,3);
% Feedforward (row 1)
G{1,1} = gains_hystory(:,1);  % Frontal
G{1,2} = gains_hystory(:,2);  % Sagittal
G{1,3} = gains_hystory(:,3);  % Transverse
% State feedback (row 2)
G{2,1} = gains_hystory(:,4:7);   % Frontal
G{2,2} = gains_hystory(:,8:11);  % Sagittal
G{2,3} = gains_hystory(:,12:13); % Transverse
% Robustifying (row 3)
G{3,1} = gains_hystory(:,14); % Frontal
G{3,2} = gains_hystory(:,15); % Sagittal
G{3,3} = gains_hystory(:,16); % Transverse

% --- PLOTTING ---
figure('Name','Control Gains (3x3)','NumberTitle','off','Color','w');
% Using 'normal' spacing to make room for ticks
tl = tiledlayout(3, 3, 'TileSpacing', 'normal', 'Padding', 'normal');
sgtitle('MRAC Gains','FontSize',14, 'FontWeight', 'bold');
axes_handles = gobjects(3,3);
for iRow = 1:3
    for iCol = 1:3
        ax = nexttile((iRow-1)*3 + iCol);
        axes_handles(iRow, iCol) = ax; 
        
        data = G{iRow,iCol};
        [N,M] = size(data);
        
        hold(ax,'on');
        for m = 1:M
            % Increased LineWidth
            plot(ax, t, data(:,m),'LineWidth', 1.8);
        end
        hold(ax,'off');
        
        % Compute min/max of the full data (actual peaks)
        if ~isempty(data)
            min_val = min(data(:));
            max_val = max(data(:));
            range = max_val - min_val;
        else
            min_val = 0; max_val = 0; range = 0;
        end

        % ---- SPECIAL CASE: State feedback (row 2) for Frontal (col1) & Sagittal (col2)
        % Force y-limits to [-10, 10] and annotate actual peaks if outside
        if iRow == 2 && (iCol == 1 || iCol == 2)
            ylims_clip = [-15, 10];
            ylim(ax, ylims_clip);
            % annotate actual peaks when they're off-scale (not visible)
            x_text = t(1) + 0.02*(t(end)-t(1)); % small left margin
            if max_val > ylims_clip(2)
                txt_top = sprintf('Actual max: %.3f (off-scale)', max_val);
                text(ax, x_text, ylims_clip(2) + 0.5, txt_top, 'FontSize', 9, ...
                    'BackgroundColor', 'w', 'EdgeColor','k', 'Margin', 2);
            end
            if min_val < ylims_clip(1)
                txt_bot = sprintf('Actual min: %.3f (off-scale)', min_val);
                text(ax, x_text, ylims_clip(1) - 0.5, txt_bot, 'FontSize', 9, ...
                    'BackgroundColor', 'w', 'EdgeColor','k', 'Margin', 2);
            end

        else
            % Original behaviour: add 10% padding (or fixed padding for flat)
            if range < 1e-6 % Handle flat lines
                padding = 0.5; % Use a fixed padding
                new_yl_min = min_val - padding;
                new_yl_max = max_val + padding;
                % Special case for line at 0, center it
                if abs(min_val) < 1e-6 && abs(max_val) < 1e-6
                   new_yl_min = -padding;
                   new_yl_max = padding;
                end
            else
                % Add 10% padding
                padding = 0.1 * range;
                new_yl_min = min_val - padding;
                new_yl_max = max_val + padding;
            end
            ylim(ax, [new_yl_min, new_yl_max]);
        end
        
        grid(ax,'on'); box(ax,'on');
        % Keep X-axis tight
        xlim(ax, [t(1), t(end)]); 
        
        % Add column titles ONLY to the top row
        if iRow == 1
            title(ax, planes{iCol}, 'FontWeight','bold','FontSize',11);
        end
        
        % Add row labels ONLY to the left column
        if iCol == 1
            ylabel(ax, contribs{iRow}, 'FontWeight','bold','FontSize',11);
        end
        
        % Remove x-axis tick labels for plots not in the bottom row
        if iRow < 3
            set(ax, 'XTickLabel', []);
        end
        
        if iRow == 2
            % Assign axis label depending on column
            switch iCol
                case 1, axis_label = 'x'; % Frontal
                case 2, axis_label = 'y'; % Sagittal
                case 3, axis_label = 'z'; % Transverse
            end
        
            % Create legend labels with axis suffix
            labels = arrayfun(@(ii) sprintf('$K_{%d,%s}$', ii, axis_label), 1:M, 'UniformOutput', false);
        
            legend(ax, labels, 'Location', 'northeastoutside', ...
                'Interpreter', 'latex', 'FontSize', 9);
        end

    end
end

linkaxes(axes_handles(:), 'x');

% Add one shared x-axis label and one shared y-axis label
xlabel(tl, 'Time (s)', 'FontWeight', 'bold', 'FontSize', 12);
ylabel(tl, 'Gain Value', 'FontWeight', 'bold', 'FontSize', 12);


% exportgraphics(gcf, 'C:\Users\chiar_zjcy1by\Desktop\Field and Service Robotics\FinalProjPlots\mrac_gains_evolution.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none')
fprintf('Plots generated successfully.\n');
end