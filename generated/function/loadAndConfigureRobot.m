function robot = loadAndConfigureRobot(urdfPath)
% This helper function loads a robot from a URDF and sets its DataFormat.
% It is intended to be called as a single extrinsic function.

    robot = importrobot(urdfPath);
    robot.DataFormat = 'column';
    
end