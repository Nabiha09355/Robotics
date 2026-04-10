function servoAngles = dh2servo(dhJointAngles)
    % dh2servo: Converts DH convention angles back to raw servomotor angles.
    % This is the exact mathematical inverse of your servo2dh function.
    
    % 1. Same Offsets as servo2dh
    offset1 = 0;       
    offset2 = pi/2;    
    offset3 = 0;       
    offset4 = 0;       

    % 2. Same Rotation Directions
    dir1 = 1;  % Base usually matches
    dir2 = -1; % Shoulder usually needs to be inverted
    dir3 = -1; % Elbow usually needs to be inverted
    dir4 = -1; % Wrist usually needs to be inverted

    % 3. Initialize Output Array
    servoAngles = zeros(1, 4);

    % 4. Apply the Inverse Transformation Formula: Servo = Dir * (DH - Offset)
    servoAngles(1) = dir1 * (dhJointAngles(1) - offset1);
    servoAngles(2) = dir2 * (dhJointAngles(2) - offset2);
    servoAngles(3) = dir3 * (dhJointAngles(3) - offset3);
    servoAngles(4) = dir4 * (dhJointAngles(4) - offset4);
    
    % 5. Wrap angles to [-pi, pi] for safe motor commands
    servoAngles = mod(servoAngles + pi, 2*pi) - pi;
end