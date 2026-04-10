%% Task 5.11: Reachable Workspace Mapping

% 1. Define Joint Angle Limits and Step Size (in radians)
step = 0.25; % Roughly 15-degree jumps

th1_range = -pi/2 : step : pi/2;       % Base
th2_range = -pi/2 : step : pi/2;       % Shoulder
th3_range = -3*pi/4 : step : 3*pi/4;   % Elbow
th4_range = -pi/2 : step : pi/2;       % Wrist

% 2. Pre-allocate memory for speed
% Calculating the total number of combinations
total_points = length(th1_range) * length(th2_range) * length(th3_range) * length(th4_range);
X = zeros(total_points, 1);
Y = zeros(total_points, 1);
Z = zeros(total_points, 1);

fprintf('Calculating %d coordinate points. Please wait...\n', total_points);

% 3. Nested Loops to calculate every combination
idx = 1;
for t1 = th1_range
    for t2 = th2_range
        for t3 = th3_range
            for t4 = th4_range
                
                % Use your verified FK function!
                [x, y, z, ~] = pincherFK([t1, t2, t3, t4]);
                
                % Optional: Ignore points that crash through the table (Z < 0)
                if z >= -20 
                    X(idx) = x;
                    Y(idx) = y;
                    Z(idx) = z;
                end
                
                idx = idx + 1;
            end
        end
    end
end

% Clean up empty rows if we ignored any points below the table
X = X(X~=0);
Y = Y(Y~=0);
Z = Z(Z~=0);

% 4. Plot the 3D Point Cloud
figure;
% scatter3(X, Y, Z, dot_size, color, fill_style)
scatter3(X, Y, Z, 2, 'b', 'filled'); 

title('Phantom X Pincher - Reachable Workspace');
xlabel('X coordinate (mm)');
ylabel('Y coordinate (mm)');
zlabel('Z coordinate (mm)');

% 'axis equal' prevents the 3D plot from looking stretched out of proportion
axis equal; 
grid on;
view(45, 30); % Sets a nice isometric viewing angle

disp('Workspace plot complete!');