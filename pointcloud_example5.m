function pointcloud_example()
   % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();


    %% Acquire device parameters 
    % Get streaming device's name
    dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

    % Extract the depth stream
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    
    % Get the intrinsics
    depth_intrinsics = depth_stream.get_intrinsics();

    %% Align the frames and then get the frames
    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    
    % Alignment is necessary as the depth cameras and RGB cameras are
    % physically separated. So, the same (x,y,z) in real world maps to
    % different (u,v) in the depth image and the color images. To build a
    % point cloud we only need depth image, but if we want the color the
    % cloud then we'll need the other image.

    % Since the two images are of different sizes, we can either align the
    % depth to color image, or the color to depth.
    % Change the argument to realsense.stream.color to align to the color
    % image.
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % Stop streaming
    pipe.stop();

    % Extract the depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);

    % Extract the color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    %% Create a point cloud using MATLAB library
    % Create a MATLAB intrinsics object
    intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    
    % Create a point cloud
    ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);
   
    figure; pcshow(ptCloud,'VerticalAxisDir','Down'); title('Full Cloud');

    %% ROI crop
    xLimits = [-0.35 0.077];
    yLimits = [0 0.16];
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

    %% ========== SEGMENT MULTIPLE BLOCKS USING EUCLIDEAN CLUSTERING ==========
    % Convert to point cloud object for clustering
    tempCloud = pointCloud(xyzAll);
    
    % Euclidean clustering to separate blocks
    minDistance = 0.01;  % 1 cm minimum distance between clusters
    [labels, numClusters] = pcsegdist(tempCloud, minDistance);
    
    fprintf('Found %d clusters (potential blocks)\n', numClusters);
    
    % Filter out small clusters (noise)
    minPointsPerBlock = 100;  % Minimum points to be considered a block
    validClusters = [];
    for i = 1:numClusters
        if sum(labels == i) >= minPointsPerBlock
            validClusters = [validClusters, i];
        end
    end
    
    numBlocks = length(validClusters);
    fprintf('Valid blocks after filtering: %d\n', numBlocks);
    
    %% ========== PROCESS EACH BLOCK ==========
    % Storage for all blocks
    allBlockData = cell(numBlocks, 1);
    
    % Define world frame once
    world_origin = [0, 0, 0];
    axis_length = 0.03;  % 3 cm for visualization
    
    for blockIdx = 1:numBlocks
        clusterLabel = validClusters(blockIdx);
        fprintf('\n========== Processing Block %d ==========\n', blockIdx);
        
        % Extract points for this block
        blockMask = labels == clusterLabel;
        xyzBlock = xyzAll(blockMask, :);
        
        %% Fit plane to TOP surface of this block
        zThreshold = prctile(xyzBlock(:,3), 80);
        topMask = xyzBlock(:,3) > zThreshold;
        xyzTopCandidate = xyzBlock(topMask, :);
        
        % Skip if not enough top points
        if size(xyzTopCandidate, 1) < 50
            fprintf('Block %d: Not enough top surface points, skipping\n', blockIdx);
            continue;
        end
        
        topCandidateCloud = pointCloud(xyzTopCandidate);
        [modelTop, inliersTop] = pcfitplane(topCandidateCloud, 0.0015);
        
        topCloud = select(topCandidateCloud, inliersTop);
        
        %% PCA-based corner detection
        xyzTop = topCloud.Location;
        xyzTop = reshape(xyzTop, [], 3);
        xyzTop = xyzTop(~any(isnan(xyzTop), 2), :);
        
        centroid = mean(xyzTop, 1);
        centered = xyzTop - centroid;
        
        [coeff, ~, ~] = pca(centered);
        
        % Ensure consistent axis orientation
        if coeff(3,3) < 0
            coeff(:,3) = -coeff(:,3);
        end
        if coeff(1,1) < 0
            coeff(:,1) = -coeff(:,1);
        end
        if coeff(2,2) < 0
            coeff(:,2) = -coeff(:,2);
        end
        
        rotated = centered * coeff;
        
        xmin = prctile(rotated(:,1), 2);
        xmax = prctile(rotated(:,1), 98);
        ymin = prctile(rotated(:,2), 2);
        ymax = prctile(rotated(:,2), 98);
        
        %% Build corners
        corners_rot = [
            xmin, ymin, 0;
            xmax, ymin, 0;
            xmax, ymax, 0;
            xmin, ymax, 0
        ];
        
        corners_temp = corners_rot * coeff' + centroid;
        
        % Project onto fitted plane
        A = modelTop.Parameters(1);
        B = modelTop.Parameters(2);
        C = modelTop.Parameters(3);
        D = modelTop.Parameters(4);
        
        for k = 1:4
            x = corners_temp(k,1);
            y = corners_temp(k,2);
            corners_temp(k,3) = -(A*x + B*y + D) / C;
        end
        corners = corners_temp;
        
        side1 = norm(corners(2,:) - corners(1,:));
        side2 = norm(corners(3,:) - corners(2,:));
        
        %% Define block frame
        block_origin = corners(1, :);
        block_x_axis = (corners(2,:) - corners(1,:)) / norm(corners(2,:) - corners(1,:));
        block_y_axis = (corners(4,:) - corners(1,:)) / norm(corners(4,:) - corners(1,:));
        block_z_axis = [A, B, C] / norm([A, B, C]);
        
        if block_z_axis(3) < 0
            block_z_axis = -block_z_axis;
        end
        
        R_block = [block_x_axis', block_y_axis', block_z_axis'];
        
        T_block_meters = eye(4);
        T_block_meters(1:3, 1:3) = R_block;
        T_block_meters(1:3, 4) = block_origin';
        
        T_block_cm = eye(4);
        T_block_cm(1:3, 1:3) = R_block;
        T_block_cm(1:3, 4) = block_origin' * 100;
        
        %% Store data
        allBlockData{blockIdx}.corners = corners;
        allBlockData{blockIdx}.origin = block_origin;
        allBlockData{blockIdx}.x_axis = block_x_axis;
        allBlockData{blockIdx}.y_axis = block_y_axis;
        allBlockData{blockIdx}.z_axis = block_z_axis;
        allBlockData{blockIdx}.T_meters = T_block_meters;
        allBlockData{blockIdx}.T_cm = T_block_cm;
        allBlockData{blockIdx}.side1 = side1;
        allBlockData{blockIdx}.side2 = side2;
        
        %% Print info
        fprintf('Block %d dimensions: %.2f cm x %.2f cm\n', blockIdx, side1*100, side2*100);
        fprintf('Origin (cm): [%.2f, %.2f, %.2f]\n', block_origin*100);
        fprintf('Transformation Matrix (cm):\n');
        disp(T_block_cm);
    end
    
    %% ========== VISUALIZATION: ALL BLOCKS TOGETHER ON ORIGINAL POINT CLOUD ==========
    
    % 3D View with original point cloud
    figure('Name', 'All Blocks - 3D View');
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(3); grid on; hold on;
    
    % Plot world frame
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            axis_length, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            0, axis_length, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            0, 0, axis_length, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    text(world_origin(1) + axis_length, world_origin(2), world_origin(3), ...
         'X_{world}', 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(world_origin(1), world_origin(2) + axis_length, world_origin(3), ...
         'Y_{world}', 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    text(world_origin(1), world_origin(2), world_origin(3) + axis_length, ...
         'Z_{world}', 'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
    
    % Plot each block's corners and frame
    for blockIdx = 1:numBlocks
        if isempty(allBlockData{blockIdx})
            continue;
        end
        
        data = allBlockData{blockIdx};
        
        % Plot corners with closed rectangle
        cornersPlot = [data.corners; data.corners(1,:)];
        plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
              'r-o', 'MarkerSize', 14, 'LineWidth', 3);
        
        % Plot block frame
        quiver3(data.origin(1), data.origin(2), data.origin(3), ...
                data.x_axis(1)*axis_length, data.x_axis(2)*axis_length, data.x_axis(3)*axis_length, ...
                'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'LineStyle', '--');
        quiver3(data.origin(1), data.origin(2), data.origin(3), ...
                data.y_axis(1)*axis_length, data.y_axis(2)*axis_length, data.y_axis(3)*axis_length, ...
                'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'LineStyle', '--');
        quiver3(data.origin(1), data.origin(2), data.origin(3), ...
                data.z_axis(1)*axis_length, data.z_axis(2)*axis_length, data.z_axis(3)*axis_length, ...
                'b', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'LineStyle', '--');
        
        % Label block frame axes
        text(data.origin(1) + data.x_axis(1)*axis_length, ...
             data.origin(2) + data.x_axis(2)*axis_length, ...
             data.origin(3) + data.x_axis(3)*axis_length, ...
             sprintf('X_{B%d}', blockIdx), 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
        text(data.origin(1) + data.y_axis(1)*axis_length, ...
             data.origin(2) + data.y_axis(2)*axis_length, ...
             data.origin(3) + data.y_axis(3)*axis_length, ...
             sprintf('Y_{B%d}', blockIdx), 'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');
        text(data.origin(1) + data.z_axis(1)*axis_length, ...
             data.origin(2) + data.z_axis(2)*axis_length, ...
             data.origin(3) + data.z_axis(3)*axis_length, ...
             sprintf('Z_{B%d}', blockIdx), 'FontSize', 10, 'Color', 'b', 'FontWeight', 'bold');
        
        % Label block number at origin
        text(data.origin(1), data.origin(2), data.origin(3) + 0.02, ...
             sprintf('Block %d', blockIdx), 'FontSize', 12, 'FontWeight', 'bold', ...
             'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
    
    hold off;
    title(sprintf('All %d Blocks with Frames (3D)', numBlocks));
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    % Top View with original point cloud
    figure('Name', 'All Blocks - Top View');
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(2); grid on; hold on;
    
    % Plot world frame (X and Y only)
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            axis_length, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            0, axis_length, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    text(world_origin(1) + axis_length, world_origin(2), world_origin(3), ...
         'X_{world}', 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(world_origin(1), world_origin(2) + axis_length, world_origin(3), ...
         'Y_{world}', 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    
    % Plot each block
    for blockIdx = 1:numBlocks
        if isempty(allBlockData{blockIdx})
            continue;
        end
        
        data = allBlockData{blockIdx};
        
        % Plot corners
        cornersPlot = [data.corners; data.corners(1,:)];
        plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
              'r-o', 'MarkerSize', 14, 'LineWidth', 3);
        
        % Plot block frame (X and Y only in top view)
        quiver3(data.origin(1), data.origin(2), data.origin(3), ...
                data.x_axis(1)*axis_length, data.x_axis(2)*axis_length, 0, ...
                'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'LineStyle', '--');
        quiver3(data.origin(1), data.origin(2), data.origin(3), ...
                data.y_axis(1)*axis_length, data.y_axis(2)*axis_length, 0, ...
                'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'LineStyle', '--');
        
        % Label block
        text(data.origin(1) + data.x_axis(1)*axis_length, ...
             data.origin(2) + data.x_axis(2)*axis_length, ...
             data.origin(3), ...
             sprintf('X_{B%d}', blockIdx), 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
        text(data.origin(1) + data.y_axis(1)*axis_length, ...
             data.origin(2) + data.y_axis(2)*axis_length, ...
             data.origin(3), ...
             sprintf('Y_{B%d}', blockIdx), 'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');
        
        text(data.origin(1), data.origin(2), data.origin(3), ...
             sprintf('B%d', blockIdx), 'FontSize', 12, 'FontWeight', 'bold', ...
             'BackgroundColor', 'white');
    end
    
    hold off;
    title(sprintf('All %d Blocks - Top View', numBlocks));
    xlabel('X (m)'); ylabel('Y (m)');
    
    %% ========== SUMMARY TABLE ==========
    fprintf('\n========== SUMMARY OF ALL BLOCKS ==========\n');
    fprintf('Block | Dimensions (cm) | Origin (cm) X, Y, Z\n');
    fprintf('------|-----------------|--------------------\n');
    for blockIdx = 1:numBlocks
        if isempty(allBlockData{blockIdx})
            continue;
        end
        data = allBlockData{blockIdx};
        fprintf('  %d   | %.2f x %.2f     | %.2f, %.2f, %.2f\n', ...
                blockIdx, data.side1*100, data.side2*100, data.origin*100);
    end
    
end