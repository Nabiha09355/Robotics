function dhJointAngles = servo2dh(servoAngles)
    % servo2dh: Converts raw servomotor angles to DH convention angles.
    
    % 1. Define the Offsets (in radians) derived from the upright pose
    offset1 = 0;       % Assuming base physical forward matches DH forward
    offset2 = pi/2;    % Difference between horizontal (DH) and upright (Servo)
    offset3 = 0;       % Elbow is straight in both conventions
    offset4 = 0;       % Wrist is straight in both conventions

    % 2. Define Rotation Directions (+1 or -1)
    % (Change to -1 if a positive servo command moves the joint opposite to your Z-axis rule)
    dir1 = 1;  % Base usually matches
    dir2 = -1; % Shoulder usually needs to be inverted
    dir3 = -1; % Elbow usually needs to be inverted
    dir4 = -1; % Wrist usually needs to be inverted

    % 3. Initialize Output Array
    dhJointAngles = zeros(1, 4);

    % 4. Apply the Transformation Formula: DH = (Dir * Servo) + Offset
    dhJointAngles(1) = (dir1 * servoAngles(1)) + offset1;
    dhJointAngles(2) = (dir2 * servoAngles(2)) + offset2;
    dhJointAngles(3) = (dir3 * servoAngles(3)) + offset3;
    dhJointAngles(4) = (dir4 * servoAngles(4)) + offset4;
    
    % 5. Wrap angles to [-pi, pi] for clean math
    dhJointAngles = mod(dhJointAngles + pi, 2*pi) - pi;
end