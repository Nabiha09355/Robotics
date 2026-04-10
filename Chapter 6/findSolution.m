function optimal_solution = findSolution(x, y, z, phi, currentConfig)
    % findSolution: Determines the optimal, collision-free IK solution
    % Inputs:
    %   x, y, z       - Desired target position (mm)
    %   phi           - Desired target orientation (rad)
    %   currentConfig - 1x4 vector of current joint angles (rad)
    % Output:
    %   optimal_solution - 1x4 vector of the chosen joint angles

    % 1. Get all mathematically possible IK solutions (Task 6.2)
    all_solutions = findJointAngles(x, y, z, phi);
    
    if isempty(all_solutions)
        disp('ERROR: Target is mathematically out of reach.');
        optimal_solution = [];
        return;
    end
    
    [num_sols, ~] = size(all_solutions);
    valid_solutions = [];

    % Define motor physical limits (150 degrees in radians for AX-12A)
    limit_rad = 150 * (pi / 180);

    % 2 & 3. Filter by Joint Limits and Collision Checks
    for i = 1:num_sols
        candidate = all_solutions(i, :);
        
        % --- Filter 1: Joint Limits ---
        % (Assuming angles are aligned with servo zero. If using DH offsets, 
        % apply your servo2dh/dh2servo conversion here first)
        if any(candidate < -limit_rad) || any(candidate > limit_rad)
            fprintf('Solution %d discarded: Exceeds physical motor limits.\n', i);
            continue; 
        end
        
        % --- Filter 2: Self-Collision ---
        % Re-use the function from Task 6.4
        if checkSelfCollision(currentConfig, candidate)
            fprintf('Solution %d discarded: Path causes self-collision.\n', i);
            continue; 
        end
        
        % If it passes both filters, add it to the valid list
        valid_solutions = [valid_solutions; candidate];
    end
    
    % If all solutions were filtered out
    if isempty(valid_solutions)
        disp('ERROR: No safe solutions exist (all violate limits or crash).');
        optimal_solution = [];
        return;
    end
    
    % 4. Find the "Optimal" Solution (Minimum Energy/Vibration)
    % Weighting factors (b_i):
    % Moving the base and shoulder requires shifting the entire weight of 
    % the arm, causing more vibration and using more energy. We penalize 
    % them heavier than the elbow and wrist.
    b1 = 4; b2 = 3; b3 = 2; b4 = 1; 
    weights = [b1, b2, b3, b4];
    
    [num_valid, ~] = size(valid_solutions);
    best_cost = inf;
    best_idx = 1;
    
    for i = 1:num_valid
        candidate = valid_solutions(i, :);
        
        % Calculate angular difference using the wrap-around trick
        % diff = mod(angle + pi, 2*pi) - pi
        angular_diff = mod((candidate - currentConfig) + pi, 2*pi) - pi;
        
        % Calculate cost: J = sum( b_i * |Delta_theta_i| )
        cost = sum(weights .* abs(angular_diff));
        
        if cost < best_cost
            best_cost = cost;
            best_idx = i;
        end
    end
    
    % Return the winning solution
    optimal_solution = valid_solutions(best_idx, :);
    
    fprintf('SUCCESS: Optimal solution selected with a movement cost of %.2f\n', best_cost);
end