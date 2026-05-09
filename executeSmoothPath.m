% executeSmoothPath.m

% 1. Define your 4 Cartesian Waypoints [X, Y, Z, Phi]
% Change these coordinates to match your actual pick and place locations
pose_A = [150, 0, 0, -pi/2];     % Pick location (Grasp)
pose_B = [150, 0, 100, -pi/2];   % Hover above pick (Pre-grasp)
pose_C = [0, 150, 100, -pi/2];   % Hover above place (Pre-place)
pose_D = [0, 150, 0, -pi/2];     % Place location (Release)

% Get current robot configuration to start IK smoothly
current_q = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];

% 2. Convert Cartesian Waypoints to Joint Space Waypoints
q_A = findSolution(pose_A(1), pose_A(2), pose_A(3), pose_A(4), current_q);
q_B = findSolution(pose_B(1), pose_B(2), pose_B(3), pose_B(4), q_A);
q_C = findSolution(pose_C(1), pose_C(2), pose_C(3), pose_C(4), q_B);
q_D = findSolution(pose_D(1), pose_D(2), pose_D(3), pose_D(4), q_C);

% Combine into a single waypoint matrix
joint_waypoints = [q_A; q_B; q_C; q_D];

% 3. Generate the Smooth Trajectory using mstraj
% Vector of maximum joint speeds (radians per second)
max_speed = [0.5, 0.5, 0.5, 0.5]; 

% Blend time (seconds) - higher number means rounder corners at B and C
t_blend = 1.0; 

% Sample interval (seconds) - determines how many points are generated
t_sample = 0.1; 

% Generate the path
% mstraj(waypoints, max_speed, t_segment, initial_pos, sample_interval, blend_time)
q_path = mstraj(joint_waypoints, max_speed, [], q_A, t_sample, t_blend);

% 4. Execute the Trajectory on the Hardware
disp('Starting smooth trajectory execution...');

% Move to the starting point A first
setPosition(q_A);
pause(2);

% Loop through the generated path and send to motors
for i = 1:size(q_path, 1)
    setPosition(q_path(i, :));
    
    % Pause for the sample interval to maintain correct speed
    pause(t_sample);
end

disp('Trajectory complete!');