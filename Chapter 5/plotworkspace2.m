%% Task 5.11: Reachable Workspace Mapping (Lab Manual Method)

% 1. Set the number of random samples (N)
N = 10000; % 10,000 points is a good balance of speed and detail

% Define Joint Limits (min and max in radians)
th1_min = -pi/2;   th1_max = pi/2;       % Base
th2_min = -pi/2;   th2_max = pi/2;       % Shoulder
th3_min = -3*pi/4; th3_max = 3*pi/4;     % Elbow
th4_min = -pi/2;   th4_max = pi/2;       % Wrist

% 2. Generate Random Angles using the manual's exact formula
% theta = theta_min + (theta_max - theta_min) * rand(N, 1)
t1 = th1_min + (th1_max - th1_min) * rand(N, 1);
t2 = th2_min + (th2_max - th2_min) * rand(N, 1);
t3 = th3_min + (th3_max - th3_min) * rand(N, 1);
t4 = th4_min + (th4_max - th4_min) * rand(N, 1);

% Pre-allocate memory for XYZ coordinates
X = zeros(N, 1);
Y = zeros(N, 1);
Z = zeros(N, 1);

disp('Calculating workspace points...');

% 3. Calculate Forward Kinematics for all N points
for i = 1:N
    % Pass each random combination into your verified function
    [x, y, z, ~] = pincherFK([t1(i), t2(i), t3(i), t4(i)]);
    X(i) = x;
    Y(i) = y;
    Z(i) = z;
end

% 4. Plot the Scatter Cloud
figure;
scatter3(X, Y, Z, 5, 'b', 'filled'); 
hold on;

% 5. Create the Outer Boundary Approximation (Convex Hull)
% The lab manual asked to use convhull and trisurf to create a shell
disp('Generating outer boundary shell...');
K = convhull(X, Y, Z);
% Plot the shell as a slightly transparent cyan surface over the dots
trisurf(K, X, Y, Z, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

title('Phantom X Pincher - Workspace & Convex Hull');
xlabel('X coordinate (mm)');
ylabel('Y coordinate (mm)');
zlabel('Z coordinate (mm)');
axis equal; 
grid on;
view(45, 30); 

% 6. Calculate Maximum Horizontal Reach
% Horizontal reach is the 2D distance from the base origin (0,0) to (X,Y)
% Formula: sqrt(x^2 + y^2)
horizontal_distances = sqrt(X.^2 + Y.^2);
max_reach = max(horizontal_distances);

fprintf('\n--- Workspace Analysis ---\n');
fprintf('Maximum Horizontal Reach: %.2f mm\n', max_reach);