function solutions = findJointAngles(x, y, z, phi)
    % findJointAngles: Calculates Inverse Kinematics for Phantom X Pincher
    % Inputs:
    %   x, y, z - Desired end-effector position in mm
    %   phi     - Desired orientation angle (theta2 + theta3 + theta4) in radians
    % Output:
    %   solutions - N x 4 matrix where each row is [theta1, theta2, theta3, theta4]

    % Physical Link Lengths (mm)
    d1 = 145; 
    a2 = 105; 
    a3 = 106; 
    a4 = 75;

    solutions = []; % Initialize empty matrix

    % Calculate the two possible base angles (Forward and Backward reach)
    th1_forward = atan2(y, x);
    th1_backward = atan2(-y, -x);

    % The planar horizontal distance r changes sign if the base faces backward
    r_forward = sqrt(x^2 + y^2);
    r_backward = -sqrt(x^2 + y^2);

    % Height relative to the shoulder joint
    s = z - d1;

    % Set up arrays to loop through both base configurations
    base_angles = [th1_forward, th1_backward];
    r_values = [r_forward, r_backward];

    for i = 1:2
        th1 = base_angles(i);
        r = r_values(i);

        % Step 2: Find coordinates of the wrist center (r_bar, s_bar)
        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);

        % Step 3: Apply Law of Cosines to find the elbow angle
        D_cos = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        % Check if the point is physically reachable (|D_cos| must be <= 1)
        if abs(D_cos) <= 1
            
            % Two possible elbow states: Elbow Down (+1) and Elbow Up (-1)
            for sign = [1, -1]
                th3 = atan2(sign * sqrt(1 - D_cos^2), D_cos);

                % Step 4: Solve for Shoulder Angle (theta2)
                th2 = atan2(s_bar, r_bar) - atan2(a3 * sin(th3), a2 + a3 * cos(th3));

                % Step 5: Solve for Wrist Angle (theta4)
                th4 = phi - th2 - th3;

                % Wrap all angles to [-pi, pi] to match the AX-12A servo limits
                th1 = mod(th1 + pi, 2*pi) - pi;
                th2 = mod(th2 + pi, 2*pi) - pi;
                th3 = mod(th3 + pi, 2*pi) - pi;
                th4 = mod(th4 + pi, 2*pi) - pi;

                % Append this valid 1x4 configuration row to the solutions matrix
                solutions = [solutions; th1, th2, th3, th4];
            end
        end
    end
end