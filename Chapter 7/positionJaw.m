function success = positionJaw(position)
    % positionJaw: Controls the linear position of the parallel gripper jaw
    % Inputs:
    %   position - Desired distance from the motor center to the jaw in mm
    % Outputs:
    %   success  - Boolean (1 if successful, 0 if invalid or failed)

    % Pull the Arbotix connection from the main Command Window
    global arb; 
    
    % Analytical Link Lengths (in mm)
    L1 = 8.68;
    L2 = 25.91;
    
    % 1. Validate the requested position (Limits of the 2R mechanism)
    min_pos = L2 - L1; % 17.23 mm
    max_pos = L1 + L2; % 34.59 mm
    
    if position < min_pos || position > max_pos
        fprintf('ERROR: Requested jaw position %.2f mm is invalid.\n', position);
        fprintf('Valid range for this mechanism is [%.2f, %.2f] mm.\n', min_pos, max_pos);
        success = 0;
        return;
    end
    
    % 2. Calculate the motor angle (theta1) using the Law of Cosines
    cos_theta1 = (L1^2 + position^2 - L2^2) / (2 * L1 * position);
    
    % Clamp rounding errors to [-1, 1] to prevent complex number errors in acos()
    cos_theta1 = max(min(cos_theta1, 1), -1); 
    theta1 = acos(cos_theta1);
    
    % 3. Experimental Offset Adjustment
    % Adjust this if your physical gap doesn't perfectly match the input!
    experimental_offset = 0.0; 
    direction = 1; 
    
    gripper_servo_angle = (direction * theta1) + experimental_offset;
    
    % 4. Safely package and send the command
    try
        % Read the CURRENT state of the first 4 joints so we don't drop the arm
        current_arm = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];
        
        % Create the full 1x5 array [Base, Shoulder, Elbow, Wrist, NEW_GRIPPER]
        full_command = [current_arm, gripper_servo_angle];
        
        % Send the full 1x5 array to the robot at a safe speed of 50
        arb.setpos(full_command, 50);
        
        success = 1;
    catch
        disp('ERROR: Failed to communicate. Make sure ''global arb;'' is initialized.');
        success = 0;
    end
end