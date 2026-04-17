% file: setPosition.m
function success = setPosition(jointAngles)
    % setPosition: Moves the 4 arm servos while maintaining current gripper state
    % Input: jointAngles - 1x4 array [q1, q2, q3, q4] in radians
    
    global arb;
    
    
    % Safety Joint Limits (Radians)
    limit_min = [-pi, -1.9, -1.8, -1.8]; 
    limit_max = [ pi,  1.9,  1.8,  1.8];
    
    % Check for out-of-bounds requests
    for i = 1:4
        if jointAngles(i) < limit_min(i) || jointAngles(i) > limit_max(i)
            fprintf('Error: Joint %d request (%.2f) exceeds safety limits.\n', i, jointAngles(i));
            success = 0;
            return;
        end
    end
    
    % Convert DH angles to Servo Ticks using Task 5.7 function
    try
        servo_ticks = dh2servo(jointAngles);
    catch
        disp('Error: dh2servo.m not found in the current folder.');
        success = 0;
        return;
    end
    
    % Hardware Execution
    try
        % Read current gripper position (Motor 5)
        current_gripper = arb.getpos(5);
        
        % Construct the 1x5 array [Base, Shoulder, Elbow, Wrist, Gripper]
        final_command = [servo_ticks(1), servo_ticks(2), servo_ticks(3), servo_ticks(4), current_gripper];
        
        % Send command with a safe speed (50)
        arb.setpos(final_command, [50,50,50,50,50]);
        success = 1;
    catch
        disp('Error: Communication failure. Check global arb and robot power.');
        success = 0;
    end
end