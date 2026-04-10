%% Task 5.11: Alternative Built-In Method

% 1. Load the digital twin you built in Task 5.9
robot = buildPincher();

% 2. Define the Position Limits for each joint (in radians)
% This tells MATLAB how far each motor is physically allowed to spin
robot.Bodies{1}.Joint.PositionLimits = [-pi/2, pi/2];       % Base
robot.Bodies{2}.Joint.PositionLimits = [-pi/2, pi/2];       % Shoulder
robot.Bodies{3}.Joint.PositionLimits = [-3*pi/4, 3*pi/4];   % Elbow
robot.Bodies{4}.Joint.PositionLimits = [-pi/2, pi/2];       % Wrist

% 3. Generate and plot the workspace automatically
figure;
generateRobotWorkspace(robot);
title('Phantom X Pincher - Built-in Workspace Generation');