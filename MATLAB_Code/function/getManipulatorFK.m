function T = getManipulatorFK(q)
%GETMANIPULATORFK Calculates the Forward Kinematics for the UR5e.
%   T = GETMANIPULATORFK(q) takes a 6x1 joint angle vector 'q' and returns
%   the 4x4 homogeneous transformation matrix 'T' for the end-effector
%   pose relative to the robot's base.

    % Use persistent variables to load the model only once for efficiency.
    persistent robotModel endEffectorName;

    if isempty(robotModel)
        disp('Loading UR5e model for FK calculation...');
        try
            % Load the robot model from the URDF file
            urdfPath = 'UR5e.urdf';
            robot = importrobot(urdfPath);
            robot.DataFormat = 'column'; % Use column vectors for joints
            
            % Store the model in the persistent variable
            robotModel = robot;
            
            % --- VERIFY THIS NAME ---
            % Use showdetails(robotModel) to find the correct name for your
            % end-effector. It might be 'tool0' or another frame.
            endEffectorName = 'robotiq_85_base_link'; 
            disp('Robot model loaded and cached for FK.');
        catch ME
            error('Failed to load the robot model. Make sure UR5e.urdf is in the path. Error: %s', ME.message);
        end
    end
    
    % Ensure the input is a column vector
    if ~iscolumn(q)
        q = q';
    end

    % Calculate the 4x4 transformation matrix for the given joint angles 'q'
    T = getTransform(robotModel, q, endEffectorName);

end