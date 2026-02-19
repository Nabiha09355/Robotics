function pointcloud_example3()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    profile = pipe.start();

    %% Acquire device parameters 
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    %% Align the frames and then get the frames
    for i = 1:5
        fs = pipe.wait_for_frames();
    end

    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    pipe.stop();

    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);

    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    %% Create a point cloud using MATLAB library
    intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy], ...
                                  [depth_intrinsics.ppx,depth_intrinsics.ppy], ...
                                  size(depth_frame));
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ColorImage=color_frame);
    
    figure; pcshow(ptCloud,'VerticalAxisDir','Down'); title('Full Cloud');

    %% ROI crop
    xLimits = [-0.1 0.1];
    yLimits = [0 0.10];
    zLimits = [0 0.385];
    roi = [xLimits yLimits zLimits];
    indices = findPointsInROI(ptCloud, roi);
    croppedCloud = select(ptCloud, indices);

    %% Denoise
    croppedCloud = pcdenoise(croppedCloud, 'NumNeighbors', 20, 'Threshold', 1);

    figure; pcshow(croppedCloud, 'VerticalAxisDir','Down'); title('Denoised Cropped Cloud');

    %% Extract XYZ cleanly from croppedCloud
    xyzAll = croppedCloud.Location;
    xyzAll = reshape(xyzAll, [], 3);
    xyzAll = xyzAll(~any(isnan(xyzAll), 2), :);
    
    fprintf('Total valid points in cropped cloud: %d\n', size(xyzAll, 1));

    %% Select top surface points using Z-threshold
    zThreshold = prctile(xyzAll(:,3), 75);
    topMask = xyzAll(:,3) > zThreshold;
    xyzTop = xyzAll(topMask, :);
    
    fprintf('Points selected for top surface: %d\n', size(xyzTop, 1));

    %% Fit plane to top surface
    topCloud = pointCloud(xyzTop);
    maxDistance = 0.002;
    
    [modelTop, inliersTop] = pcfitplane(topCloud, maxDistance);
    topCloud = select(topCloud, inliersTop);
    
    fprintf('Inliers in plane fit: %d\n', length(inliersTop));

    figure; pcshow(topCloud, 'VerticalAxisDir','Down'); axis equal; title('Top Surface');

    %% Get final top points
    xyzTop = topCloud.Location;
    xyzTop = reshape(xyzTop, [], 3);
    xyzTop = xyzTop(~any(isnan(xyzTop), 2), :);
    
    %% Center the data
    centroid = mean(xyzTop, 1);
    centered = xyzTop - centroid;
    
    %% Create orthonormal basis on the plane
    [coeff, ~, ~] = pca(centered);
    
    normal = coeff(:,3);
    if normal(3) < 0
        normal = -normal;
    end
    
    u = coeff(:,1);
    v = coeff(:,2);
    
    % Project points onto plane using u,v basis
    pts2D = [centered * u, centered * v];
    
    %% IMPROVED: Find convex hull, then fit minimum area rectangle to hull
    % Step 1: Get convex hull of 2D points
    try
        hullIdx = convhull(pts2D(:,1), pts2D(:,2));
        hullPts = pts2D(hullIdx, :);
        
        fprintf('Convex hull has %d vertices\n', length(hullIdx));
        
        % Step 2: For each edge of the convex hull, try aligning the rectangle to it
        minArea = inf;
        bestAngle = 0;
        bestBounds = [0 0 0 0];
        
        numEdges = length(hullIdx) - 1; % -1 because last point = first point
        
        for i = 1:numEdges
            % Get edge vector
            edge = hullPts(i+1, :) - hullPts(i, :);
            
            % Calculate angle of this edge
            theta = atan2(edge(2), edge(1));
            
            % Rotation matrix to align this edge with x-axis
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            
            % Rotate all points (not just hull)
            pts_rot = pts2D * R;
            
            % Get bounding box
            min_x = min(pts_rot(:,1));
            max_x = max(pts_rot(:,1));
            min_y = min(pts_rot(:,2));
            max_y = max(pts_rot(:,2));
            
            currentArea = (max_x - min_x) * (max_y - min_y);
            
            if currentArea < minArea
                minArea = currentArea;
                bestAngle = theta;
                bestBounds = [min_x, max_x, min_y, max_y];
            end
        end
        
        % Also try angles perpendicular to hull edges (can help with rectangles)
        for i = 1:numEdges
            edge = hullPts(i+1, :) - hullPts(i, :);
            theta = atan2(edge(2), edge(1)) + pi/2; % perpendicular
            
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            pts_rot = pts2D * R;
            
            min_x = min(pts_rot(:,1));
            max_x = max(pts_rot(:,1));
            min_y = min(pts_rot(:,2));
            max_y = max(pts_rot(:,2));
            
            currentArea = (max_x - min_x) * (max_y - min_y);
            
            if currentArea < minArea
                minArea = currentArea;
                bestAngle = theta;
                bestBounds = [min_x, max_x, min_y, max_y];
            end
        end
        
    catch ME
        warning('Convex hull failed: %s. Using brute force method.', ME.message);
        
        % Fallback to brute force
        minArea = inf;
        bestAngle = 0;
        bestBounds = [0 0 0 0];
        
        angles = 0:0.5:180;
        for theta_deg = angles
            theta = deg2rad(theta_deg);
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            pts_rot = pts2D * R;
            
            min_x = min(pts_rot(:,1));
            max_x = max(pts_rot(:,1));
            min_y = min(pts_rot(:,2));
            max_y = max(pts_rot(:,2));
            
            currentArea = (max_x - min_x) * (max_y - min_y);
            
            if currentArea < minArea
                minArea = currentArea;
                bestAngle = theta;
                bestBounds = [min_x, max_x, min_y, max_y];
            end
        end
    end
    
    fprintf('Best angle found: %.2f degrees\n', rad2deg(bestAngle));
    fprintf('Minimum area: %.6f m^2\n', minArea);
    
    %% Reconstruct Corners
    b_xmin = bestBounds(1); 
    b_xmax = bestBounds(2);
    b_ymin = bestBounds(3); 
    b_ymax = bestBounds(4);
    
    corners_best_rot = [
        b_xmin, b_ymin;
        b_xmax, b_ymin;
        b_xmax, b_ymax;
        b_xmin, b_ymax
    ];

    % Rotate back to plane frame
    R_best = [cos(bestAngle) -sin(bestAngle); sin(bestAngle) cos(bestAngle)];
    corners_2d = corners_best_rot * R_best'; 
    
    % Convert back to 3D using the u,v basis
    corners_temp = centroid + corners_2d(:,1) * u' + corners_2d(:,2) * v';

    %% Project corners onto actual fitted plane
    A = modelTop.Parameters(1);
    B = modelTop.Parameters(2);
    C = modelTop.Parameters(3);
    D = modelTop.Parameters(4);

    corners = zeros(4, 3);
    for k = 1:4
        x = corners_temp(k,1);
        y = corners_temp(k,2);
        corners(k,:) = [x, y, -(A*x + B*y + D) / C];
    end

    %% Close the rectangle for plotting
    cornersPlot = [corners; corners(1,:)];

    % Compute side lengths
    side1 = norm(corners(2,:) - corners(1,:));
    side2 = norm(corners(3,:) - corners(2,:));
    side3 = norm(corners(4,:) - corners(3,:));
    side4 = norm(corners(1,:) - corners(4,:));
    
    fprintf('\nCorner side lengths:\n');
    fprintf('  Side 1-2: %.4f m\n', side1);
    fprintf('  Side 2-3: %.4f m\n', side2);
    fprintf('  Side 3-4: %.4f m\n', side3);
    fprintf('  Side 4-1: %.4f m\n', side4);
    fprintf('\nCorners (camera frame XYZ):\n');
    disp(corners);

    %% Visualize
    figure
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(3); grid on; hold on;
    plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
          'r-o', 'MarkerSize', 14, 'LineWidth', 3);
    hold off;
    title('Final Detected Corners (3D)');

    figure
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(2); grid on; hold on;
    plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
          'r-o', 'MarkerSize', 14, 'LineWidth', 3);
    hold off;
    title('Final Detected Corners (Top View)');
    
    %% IMPROVED 2D visualization with convex hull
    figure;
    R_best_final = [cos(bestAngle) -sin(bestAngle); sin(bestAngle) cos(bestAngle)];
    pts_rot_final = pts2D * R_best_final;
    
    % Plot all points
    plot(pts_rot_final(:,1), pts_rot_final(:,2), 'b.', 'MarkerSize', 2); hold on;
    
    % Plot convex hull if it exists
    if exist('hullIdx', 'var')
        hullPts_rot = pts2D(hullIdx, :) * R_best_final;
        plot(hullPts_rot(:,1), hullPts_rot(:,2), 'g-', 'LineWidth', 1.5);
    end
    
    % Plot rectangle
    corners_rot_final = corners_2d * R_best_final';
    corners_rot_plot = [corners_rot_final; corners_rot_final(1,:)];
    plot(corners_rot_plot(:,1), corners_rot_plot(:,2), 'r-o', 'LineWidth', 2, 'MarkerSize', 10);
    
    axis equal; grid on;
    title('2D Projection: Points (blue), Hull (green), Rectangle (red)');
    xlabel('U (m)'); ylabel('V (m)');
    legend('Points', 'Convex Hull', 'Fitted Rectangle', 'Location', 'best');
end
