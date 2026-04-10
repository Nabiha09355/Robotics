function [x, y, z, R] = findPincher(arb)
    % findPincher: Queries the live servomotors and calculates the end-effector pose.
    %
    % Inputs:
    %   arb - The Arbotix connection object.
    %
    % Outputs:
    %   x, y, z - Cartesian position of the gripper in mm.
    %   R       - 3x3 Rotation matrix representing end-effector orientation.

    % 1. Read the raw servo angles from the physical arm [cite: 1098]
    % (Querying motors 1 through 4)
    raw_angles = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];

    % 2. Convert the raw servo angles to DH angles [cite: 1099]
    % (Using the function you built in Task 5.7)
    dh_angles = servo2dh(raw_angles);

    % 3. Calculate the Forward Kinematics [cite: 1099]
    % (Using the function you built in Task 5.6)
    [x, y, z, R] = pincherFK(dh_angles);
    
    % Optional: Print the results to the Command Window for easy reading
    fprintf('Current End-Effector Position (mm): X=%.2f, Y=%.2f, Z=%.2f\n', x, y, z);
end