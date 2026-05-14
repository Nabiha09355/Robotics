%% Task 6.6: Accuracy Calculation and Table Generation

% 1. Define the 5 Target Points (x, y, z, phi)
% Change these to the 5 random points you selected in your workspace
targets = [
    150,   0, 100, 0;  % Point 1
    100, 100, 150, 0;  % Point 2
     50, 150,  80, 0;  % Point 3
      0, 200, 200, 0;  % Point 4
    120, -80,  50, 0   % Point 5
];

% 2. Input your physical measurements here (in mm)
% Replace these matrix rows with your actual ruler measurements [x, y, z]
% Assuming 3 trials from different starting configurations per point
measurements = {
    [148,   2,  98;  149,  -1,  99;  147,   1,  97], % Trials for P1
    [ 98, 102, 147;   99, 101, 148;   97,  98, 146], % Trials for P2
    [ 51, 148,  82;   49, 149,  79;   52, 147,  81], % Trials for P3
    [ -2, 198, 196;    1, 197, 195;   -1, 199, 197], % Trials for P4
    [122, -78,  48;  121, -79,  49;  123, -77,  47]  % Trials for P5
};

fprintf('--- Task 6.6: Positioning Accuracy Table ---\n\n');
fprintf('Target (X, Y, Z)\tTrial\tMeasured (X, Y, Z)\tEuclidean Error (mm)\n');
fprintf('----------------------------------------------------------------------\n');

total_mean_errors = zeros(5, 1);

% 3. Calculate Euclidean Error and Print Table
for i = 1:5
    target_xyz = targets(i, 1:3);
    trials = measurements{i};
    point_errors = zeros(3, 1);
    
    for trial_num = 1:3
        measured_xyz = trials(trial_num, :);
        
        % Euclidean Error Formula: sqrt((x_c - x_a)^2 + (y_c - y_a)^2 + (z_c - z_a)^2)
        err = norm(target_xyz - measured_xyz);
        point_errors(trial_num) = err;
        
        fprintf('[%4d, %4d, %4d]\t  %d  \t[%4d, %4d, %4d]\t\t%.2f\n', ...
            target_xyz(1), target_xyz(2), target_xyz(3), ...
            trial_num, ...
            measured_xyz(1), measured_xyz(2), measured_xyz(3), ...
            err);
    end
    
    mean_error = mean(point_errors);
    total_mean_errors(i) = mean_error;
    fprintf('>>> Mean Absolute Error for Point %d: %.2f mm <<<\n', i, mean_error);
    fprintf('----------------------------------------------------------------------\n');
end

fprintf('\nGLOBAL ACCURACY: The overall mean error across all points is %.2f mm\n', mean(total_mean_errors));