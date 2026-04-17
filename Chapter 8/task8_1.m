global arb;

% 1. Define Waypoints and Parameters
waypoints = [
    150,   0, 140;   
    106, 106, 140;   
      0, 150, 140    
];
max_velocity = [60, 60, 60]; 
dt = 0.05;                   
t_blend = 0.5;               
q_start = waypoints(1, :); 

% 2. Generate Cartesian Path
disp('Generating mathematical trajectory...');
smooth_path = mstraj(waypoints, max_velocity, [], q_start, dt, t_blend);

% --- NEW: Initialize the Data Storage Matrix ---
num_steps = size(smooth_path, 1);
Q_matrix = zeros(num_steps, 4); % N rows, 4 columns for our 4 servos

% 3. Execution Setup
disp('Executing smooth trajectory on hardware...');
phi = -pi/2; 
current_q = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];

% 4. Trajectory Loop
for i = 1:num_steps
    target_xyz = smooth_path(i, :);
    
    % Calculate IK for the current step
    target_joint_angles = findSolution(target_xyz(1), target_xyz(2), target_xyz(3), phi, current_q);
    
    % --- NEW: Record the calculated angles for plotting ---
    Q_matrix(i, :) = target_joint_angles; 
    
    % Send command to hardware
    if setPosition(target_joint_angles)
        current_q = target_joint_angles; 
        pause(dt); 
    else
        fprintf('\nTrajectory interrupted at Step %d.\n', i);
        break; 
    end
end
disp('Physical Trajectory complete. Generating plots...');

% ==========================================
% 5. CALCULATE AND PLOT THE DIGITAL DATA
% ==========================================

% ==========================================
% 5. CALCULATE, CLEAN, AND PLOT
% ==========================================

% Create time vectors
t = (0:num_steps-1) * dt; 
t_vel = t(1:end-1); 
t_acc = t_vel(1:end-1); 

% Calculate Raw Velocity and Acceleration
Qd = diff(Q_matrix) / dt;
Qdd = diff(Qd) / dt;

% --- NEW: Apply a moving average filter to remove the math noise ---
Qd_clean = smoothdata(Qd, 'movmean', 5);
Qdd_clean = smoothdata(Qdd, 'movmean', 5);

% Generate the Figure
figure('Name', 'Task 8.1: Joint Space Trajectory', 'Color', 'w');

% Subplot 1: Position (Remains untouched)
subplot(3,1,1);
plot(t, Q_matrix, 'LineWidth', 1.5);
title('Joint Positions (LSPB Trajectory)');
ylabel('Position (rad)');
legend('Base', 'Shoulder', 'Elbow', 'Wrist', 'Location', 'best');
grid on;

% Subplot 2: Velocity (Using Clean Data)
subplot(3,1,2);
plot(t_vel, Qd_clean, 'LineWidth', 1.5);
title('Joint Velocities (Bell-Shape)');
ylabel('Velocity (rad/s)');
grid on;

% Subplot 3: Acceleration (Using Clean Data)
subplot(3,1,3);
plot(t_acc, Qdd_clean, 'LineWidth', 1.5);
title('Joint Accelerations');
ylabel('Accel (rad/s^2)');
xlabel('Time (seconds)');
grid on;