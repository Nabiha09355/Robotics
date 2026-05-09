function isCollision = checkSelfCollision(initial_angles, final_angles)
    % checkSelfCollision: Evaluates if the arm will hit itself during motion.
    % Inputs:
    %   initial_angles - 1x4 vector of starting joint angles (radians)
    %   final_angles   - 1x4 vector of ending joint angles (radians)
    % Output:
    %   isCollision    - Boolean (true if it crashes, false if safe)

    % 1. Load the digital twin with the collision geometries added
    robot = buildPincher();
    robot = addPincherCollisions(robot);

    % 2. Define the number of intermediate samples to check along the path
    num_samples = 50; 
    
    % Initialize collision flag as false (safe)
    isCollision = false;

    % 3. Sample the path between the initial and final configuration
    fprintf('Checking trajectory for self-collisions...\n');
    
    for i = 1:num_samples
        % Interpolate to find the intermediate joint angles
        % fraction goes from 0 (start) to 1 (end)
        fraction = i / num_samples; 
        current_config = initial_angles + fraction * (final_angles - initial_angles);
        
        % 4. Evaluate the configuration using MATLAB's built-in collision checker
        % checkCollision returns an array of collisions. If any element is 1, a crash occurred.
        collision_status = checkCollision(robot, current_config, 'SkippedSelfCollisions', 'adjacent');
        
        if any(collision_status)
            isCollision = true;
            fprintf('WARNING: Self-collision detected at path step %d!\n', i);
            fprintf('Dangerous Angles: [%.2f, %.2f, %.2f, %.2f]\n', current_config);
            break; % Stop checking, the path is already invalid
        end
    end
    
    if ~isCollision
        disp('Path is clear! No self-collisions detected.');
    end
end