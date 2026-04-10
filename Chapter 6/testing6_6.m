%% Task 6.6: Automated Physical Testing Script

% 1. Define your 5 Test Points [X, Y, Z, Phi]
% I have selected 5 points spread across the safe workspace. 
% You can change these if you want to test different areas!
test_points = [
    150,   0, 100, 0;  % Point 1: Straight forward, mid-height
    100, 100, 120, 0;  % Point 2: Forward-left
     80,-100,  80, 0;  % Point 3: Forward-right, low
      0, 180, 150, 0;  % Point 4: Far left, high up
    120,  50,  50, 0   % Point 5: Forward-left, very low
];

measured_results = zeros(5, 3); % Matrix to store your [X, Y, Z] ruler inputs
errors = zeros(5, 1);

fprintf('=== STARTING PHYSICAL ACCURACY TEST ===\n\n');

% Loop through each of the 5 points
for i = 1:5
    target = test_points(i, :);
    fprintf('\n--- Testing Point %d: Target [X=%.1f, Y=%.1f, Z=%.1f] ---\n', i, target(1), target(2), target(3));
    
    % Get current position
    current_servo = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];
    current_dh = servo2dh(current_servo);
    
    % Calculate IK
    target_dh = findSolution(target(1), target(2), target(3), target(4), current_dh);
    
    if isempty(target_dh)
        fprintf('SKIPPING: No safe path to this point.\n');
        continue;
    end
    
    % Convert to servo commands and add current gripper position
    target_servo = dh2servo(target_dh);
    final_cmd = [target_servo, arb.getpos(5)];
    
    % EXECUTE MOTION (Speed 50)
    fprintf('Moving arm... Keep hand near power plug!\n');
    arb.setpos(final_cmd, 50);
    
    % Wait for the arm to finish moving (adjust this pause if needed)
    pause(4); 
    
    % PROMPT USER FOR RULER MEASUREMENTS
    disp('Measurement Time! Use your ruler to find the physical center of the gripper horn.');
    % The script will pause here until you type your measurements and hit Enter
    user_input = input('Type your measurement as [X, Y, Z] and press Enter: ');
    
    % Store data and calculate error
    measured_results(i, :) = user_input;
    errors(i) = norm(target(1:3) - user_input);
    fprintf('Recorded Error for Point %d: %.2f mm\n', i, errors(i));
end

%% Print Final Report Table
fprintf('\n\n================ FINAL LAB REPORT TABLE ================\n');
fprintf('Target (X,Y,Z)\t\tMeasured (X,Y,Z)\tEuclidean Error\n');
fprintf('--------------------------------------------------------\n');
for i = 1:5
    t = test_points(i,:);
    m = measured_results(i,:);
    fprintf('[%3d, %3d, %3d]\t\t[%3d, %3d, %3d]\t\t%.2f mm\n', ...
        t(1), t(2), t(3), m(1), m(2), m(3), errors(i));
end
fprintf('--------------------------------------------------------\n');
fprintf('GLOBAL MEAN ERROR: %.2f mm\n', mean(errors));
fprintf('========================================================\n');