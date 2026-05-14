%% Task 5.10: Forward Kinematics Verification

% 1. Load your digital twin from Task 5.9
robot = buildPincher();

% 2. Pick a random set of test angles (DH convention, in radians)
test_angles = [pi/6, -pi/4, pi/3, -pi/6]; 

% --- 3. CALCULATE USING YOUR MANUAL FUNCTION (Task 5.6) ---
[x, y, z, R_manual] = pincherFK(test_angles);

% Reconstruct the full 4x4 Homogeneous Transformation Matrix
T_manual = eye(4);
T_manual(1:3, 1:3) = R_manual;
T_manual(1:3, 4)   = [x; y; z];

% --- 4. CALCULATE USING MATLAB'S BUILT-IN FUNCTION ---
% The getTransform function calculates the frame of 'link4' relative to 'base'
% Make sure 'link4' matches the name you gave the final link in your buildPincher function
T_matlab = getTransform(robot, test_angles, 'link4', 'base');

% --- 5. COMPARE THE RESULTS ---
disp('--- Manual FK Matrix (pincherFK) ---');
disp(T_manual);

disp('--- MATLAB Digital Twin Matrix (getTransform) ---');
disp(T_matlab);

% 6. Calculate the error (difference between the two)
% We round to 4 decimal places to ignore tiny floating-point math errors
error_matrix = round(T_manual - T_matlab, 4);
disp('--- Error Matrix (Should be all zeros) ---');
disp(error_matrix);