%% Task 5.11: Reachable Workspace with Random Sampling & Convex Hull

% 1. Set the number of samples (N)
% 15,000 provides a very dense, clear cloud without slowing down MATLAB
N = 15000; 

% 2. Define Joint Limits (radians) based on physical constraints
% Base: -90 to 90 | Shoulder: -90 to 90 | Elbow: -135 to 135 | Wrist: -90 to 90
limits = [
    -pi/2,   pi/2;    % Joint 1
    -pi/2,   pi/2;    % Joint 2
    -3*pi/4, 3*pi/4;  % Joint 3
    -pi/2,   pi/2     % Joint 4
];

% 3. Generate Random Angles using the manual's exact formula:
% theta = theta_min + (theta_max - theta_min) * rand(N,1)
t1 = limits(1,1) + (limits(1,2) - limits(1,1)) * rand(N,1);
t2 = limits(2,1) + (limits(2,2) - limits(2,1)) * rand(N,1);
t3 = limits(3,1) + (limits(3,2) - limits(3,1)) * rand(N,1);
t4 = limits(4,1) + (limits(4,2) - limits(4,1)) * rand(N,1);

% 4. Pre-allocate XYZ coordinates
X = zeros(N, 1);
Y = zeros(N, 1);
Z = zeros(N, 1);

fprintf('Generating %d random samples. This may take a moment...\n', N);

% 5. Calculate Forward Kinematics for every sample
for i = 1:N
    [x, y, z, ~] = pincherFK([t1(i), t2(i), t3(i), t4(i)]);
    X(i) = x;
    Y(i) = y;
    Z(i) = z;
end

% 6. Calculate Maximum Horizontal Reach
% Reach = sqrt(x^2 + y^2)
horizontal_reach = sqrt(X.^2 + Y.^2);
[max_val, max_idx] = max(horizontal_reach);

% 7. Plotting for High Clarity
figure('Color', 'w', 'Name', 'Phantom X Reachable Workspace');

% Plot the random point cloud
scatter3(X, Y, Z, 3, 'k', 'filled', 'MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1); 
hold on;

% Create and plot the Outer Boundary (Convex Hull)
% This creates a "skin" over the points
[k, vol] = convhull(X, Y, Z);
trisurf(k, X, Y, Z, 'FaceColor', 'cyan', 'FaceAlpha', 0.4, 'EdgeColor', 'none');

% Highlight the point of Maximum Reach
plot3(X(max_idx), Y(max_idx), Z(max_idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(X(max_idx), Y(max_idx), Z(max_idx), ['  Max Reach: ', num2str(round(max_val)), 'mm'], 'FontSize', 12, 'Color', 'r');

% Formatting
title('Task 5.11: Reachable Workspace & Convex Hull Boundary');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
axis equal;
view(45, 20);
camlight; % Adds lighting to make the 3D surface look better
lighting gouraud;

fprintf('--- Workspace Results ---\n');
fprintf('Maximum Horizontal Reach: %.2f mm\n', max_val);
fprintf('Total Reachable Volume: %.2f cubic mm\n', vol);