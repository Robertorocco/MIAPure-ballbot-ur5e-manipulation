% --- Modified Script to Process Bag Data for Simscape ---
%
% This script reads a ROS2 bag file, extracts laggy joint position data,
% re-processes it into a smooth trajectory, and prepares it for a Simscape simulation.
% clear;
% clc;
% close all;

%% Giving mesh general path:
%open_system('manipulator.slx');
%blocks = find_system('manipulator', 'MaskType', 'File Solid')
basePath = pwd;
ur5eRoot  = fullfile(basePath, 'ros_packages', 'ur_description', 'meshes', 'ur5e', 'visual');
gripperRoot = fullfile(basePath, 'ros_packages', 'robotiq_description', 'meshes', 'visual', '2f_85');
meshMap = containers.Map;
% UR5e Links 
meshMap([modelName '/base_link_inertia/Visual'])       = fullfile(ur5eRoot, 'base.dae');
meshMap([modelName '/shoulder_link/Visual'])           = fullfile(ur5eRoot, 'shoulder.dae');
meshMap([modelName '/upper_arm_link/Visual'])          = fullfile(ur5eRoot, 'upperarm.dae');
meshMap([modelName '/forearm_link/Visual'])            = fullfile(ur5eRoot, 'forearm.dae');
meshMap([modelName '/wrist_1_link/Visual'])            = fullfile(ur5eRoot, 'wrist1.dae');
meshMap([modelName '/wrist_2_link/Visual'])            = fullfile(ur5eRoot, 'wrist2.dae');
meshMap([modelName '/wrist_3_link/Visual'])            = fullfile(ur5eRoot, 'wrist3.dae');
% Robotiq 2F-85 Gripper Links
meshMap([modelName '/robotiq_85_base_link/Visual'])               = fullfile(gripperRoot, 'robotiq_base.dae');
meshMap([modelName '/robotiq_85_left_finger_link/Visual'])        = fullfile(gripperRoot, 'left_finger.dae');
meshMap([modelName '/robotiq_85_left_finger_tip_link/Visual'])    = fullfile(gripperRoot, 'left_finger_tip.dae');
meshMap([modelName '/robotiq_85_left_inner_knuckle_link/Visual']) = fullfile(gripperRoot, 'left_inner_knuckle.dae');
meshMap([modelName '/robotiq_85_left_knuckle_link/Visual'])       = fullfile(gripperRoot, 'left_knuckle.dae');
meshMap([modelName '/robotiq_85_right_finger_link/Visual'])       = fullfile(gripperRoot, 'right_finger.dae');
meshMap([modelName '/robotiq_85_right_finger_tip_link/Visual'])   = fullfile(gripperRoot, 'right_finger_tip.dae');
meshMap([modelName '/robotiq_85_right_inner_knuckle_link/Visual'])= fullfile(gripperRoot, 'right_inner_knuckle.dae');
meshMap([modelName '/robotiq_85_right_knuckle_link/Visual'])      = fullfile(gripperRoot, 'right_knuckle.dae');
meshMap('manipulator/robotiq_85_right_knuckle_link/Visual')       = fullfile(gripperRoot, 'right_knuckle.dae');
if ~bdIsLoaded(modelName)
    load_system(modelName);
end
% Get all File Solid blocks
blocks = find_system(modelName, 'MaskType', 'File Solid');
% Loop and assign
for i = 1:length(blocks)
    blockPath = blocks{i};
    if isKey(meshMap, blockPath)
        set_param(blockPath, 'ExtGeomFileName', meshMap(blockPath));
        fprintf('Set mesh for %s -> %s\n', blockPath, meshMap(blockPath));
    else
        warning('No mesh mapping found for block %s', blockPath);
    end
end

%% --- Configuration ---
% Specify the path to your ROS2 bag file.
bagFilePath = 'complicated_bag.db3'; 
% Specify the topic to read from.
topicName = '/joint_states';
% Specify your desired simulation sample rate in Hertz.
simulationSampleRate = 100; % 100 Hz is a good starting point
% Specify the trajectory speed scaling factor. >1 for slower, <1 for faster.
traj_scaling_factor = 2.0; % e.g., 2.0 makes the trajectory twice as slow.
%TRAJECOTRY SPEED MOVED ON SIMULINK SINCE LOOKUP TABLE
%traj_scaling_factor = 0.5;

%% --- Check if Bag File Exists ---
if ~isfile(bagFilePath)
    error('Bag file not found at the specified path: %s', bagFilePath);
end
fprintf('Reading bag file: %s\n', bagFilePath);

%% --- Read and Process Bag Data ---
% Create a ros2bagreader object.
bagReader = ros2bagreader(bagFilePath);
% Select the specific topic from the bag.
jointStatesSelection = select(bagReader, 'Topic', topicName);
% Read all messages from the selection.
allMessages = readMessages(jointStatesSelection);
numMessages = numel(allMessages);
if numMessages == 0
    error('No messages found on topic %s in the bag file.', topicName);
end
fprintf('Found %d messages on topic %s.\n', numMessages, topicName);
% Get the list of joint names from the first message.
jointNames = allMessages{1}.name;
numJoints = numel(jointNames);
% --- IMPORTANT: Check if the number of joints is 7 ---
if numJoints ~= 7
    warning('Expected 7 joints but found %d. Verify your bag file.', numJoints);
end
% Prepare a simple matrix to hold the position data for easier processing.
timeVector = zeros(numMessages, 1);
rawJointPositions = zeros(numMessages, numJoints);
fprintf('Extracting raw data for %d joints...\n', numJoints);
% Create a map for joint name to column index for robust ordering.
jointIndexMap = containers.Map(jointNames, 1:numJoints);
% Extract the data from each message.
for i = 1:numMessages
    msg = allMessages{i};
    timeVector(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    for j = 1:numel(msg.name)
        currentJointName = msg.name{j};
        if isKey(jointIndexMap, currentJointName)
            colIdx = jointIndexMap(currentJointName);
            rawJointPositions(i, colIdx) = msg.position(j);
        end
    end
end
% Normalize the time vector to start from 0.
timeVector = timeVector - timeVector(1);

%% --- 1. Generate a New, Smooth Trajectory 📈 ---
% Smooth the trajectory using spline interpolation to fix the low RTF recording.
fprintf('Resampling and smoothing the laggy data...\n');
totalDuration = timeVector(end);
time_smooth = (0 : 1/simulationSampleRate : totalDuration)';
positions_smooth = zeros(length(time_smooth), numJoints);
for j = 1:numJoints
    positions_smooth(:, j) = interp1(timeVector, rawJointPositions(:, j), time_smooth, 'spline');
end
fprintf('New smooth trajectory generated with %d points.\n', length(time_smooth));

%% --- 2. Remove Stationary Periods 🚶‍♂️---
% Eliminate consecutive duplicate joint positions to remove idle time.
fprintf('Removing stationary periods from the trajectory...\n');
tolerance = 1e-6; % Tolerance to detect a change in position.
rowsToKeep = [true; vecnorm(diff(positions_smooth), 2, 2) > tolerance];

% --- CORRECTED LOGIC STARTS HERE ---
% Keep only the position data for non-stationary points.
positions_dedup = positions_smooth(rowsToKeep, :);

% Generate a new, compact time vector based on the number of REMAINING points.
% This correctly removes the time gaps.
num_final_points = size(positions_dedup, 1);
new_duration = (num_final_points - 1) / simulationSampleRate;
time_dedup = (0 : 1/simulationSampleRate : new_duration)';
% --- CORRECTED LOGIC ENDS HERE ---

fprintf('Removed %d stationary points. New point count: %d.\n', length(rowsToKeep) - sum(rowsToKeep), length(time_dedup));
fprintf('Original duration was %.2fs. New duration after removing idle time is %.2fs.\n', totalDuration, new_duration);
%% --- 3. Scale Trajectory Speed 🐢---
% Apply a scaling factor to slow down or speed up the trajectory.
fprintf('Scaling trajectory time with a factor of %.2f...\n', traj_scaling_factor);
time_final = time_dedup * traj_scaling_factor;
positions_final = positions_dedup;
fprintf('Final trajectory duration is now %.2f seconds.\n', time_final(end));

%% --- Prepare Data for Simscape 🚀 ---
% The 'From Workspace' block in Simulink requires a specific format:
% a matrix where the first column is time, and the subsequent columns are data.
simscape_trajectory = [time_final, positions_final];
manipulator_time_vector = time_final;
manipulator_position_data = positions_final;

%% --- Plotting Comparison ---
fprintf('Plotting results for comparison...\n');
figure('Name', 'Raw vs. Processed Trajectory');
% Create a tiled layout for subplots.
tiledlayout(2, 1);
% --- Plot 1: Raw, Laggy Data ---
nexttile;
plot(timeVector, rawJointPositions);
grid on;
title('Original Laggy Data from rosbag');
xlabel('Time (s)');
ylabel('Joint Position (rad)');
legend(jointNames, 'Interpreter', 'none', 'Location', 'eastoutside');
% --- Plot 2: Final Processed Data ---
nexttile;
plot(time_final, positions_final);
grid on;
title('Final Processed Trajectory for Simscape');
xlabel('Time (s)');
ylabel('Joint Position (rad)');
legend(jointNames, 'Interpreter', 'none', 'Location', 'eastoutside');
fprintf('Done.\n');

%% --- Display Initial Conditions for Simscape ---
% This part is useful for setting the initial state of your Simscape model.
initial_names = allMessages{1}.name
initial_positions_rad = rawJointPositions(1, :)'

%% --- 6. Create the Rigid Body Tree --- % 
% Path to your URDF (adjust if it's in another folder)
urdfPath = fullfile(basePath, "UR5e.urdf");
% Import only the rigid body tree (geometry optional)
robot = importrobot(urdfPath, "MeshPath", "");
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.81];