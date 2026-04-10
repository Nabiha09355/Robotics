%% Task 6.2: Inverse Kinematics Verification

% 1. Define a target point in the workspace
% Example: 150mm forward, 100mm left, 120mm high, with gripper horizontal (phi = 0)
target_x = 20;
target_y = 20;
target_z = 120;
target_phi = 0; 

fprintf('TARGET POSE: X=%.1f, Y=%.1f, Z=%.1f, Phi=%.2f rad\n\n', target_x, target_y, target_z, target_phi);

% 2. Get all possible IK solutions (Should be an N x 4 matrix)
ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi);

[num_solutions, ~] = size(ik_solutions);
fprintf('Found %d mathematically valid solutions.\n', num_solutions);
disp('--------------------------------------------------');

% 3. Loop through each solution row and verify it with Forward Kinematics
for i = 1:num_solutions
    angles = ik_solutions(i, :);
    
    % Pass the 4 angles into your FK function from Chapter 5
    [fk_x, fk_y, fk_z, ~] = pincherFK_chp6(angles);
    
    % Print the results
    fprintf('Solution %d Joint Angles (rad): [%.4f, %.4f, %.4f, %.4f]\n', i, angles(1), angles(2), angles(3), angles(4));
    fprintf('   FK Verification -> X: %6.1f | Y: %6.1f | Z: %6.1f\n', fk_x, fk_y, fk_z);
    
    % Check error margin
    error = norm([target_x, target_y, target_z] - [fk_x, fk_y, fk_z]);
    if error < 0.1
        fprintf('   STATUS: SUCCESS (Error: %.4f mm)\n\n', error);
    else
        fprintf('   STATUS: FAILED (Error: %.4f mm)\n\n', error);
    end
end