function pointcloud_example2()
    data = load('mpointcloud1.mat');
    ptCloud = data.ptCloud;

   
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

    %% --- Extract XYZ cleanly from croppedCloud ---
    xyzAll = croppedCloud.Location;
    xyzAll = reshape(xyzAll, [], 3);
    xyzAll = xyzAll(~any(isnan(xyzAll), 2), :);

    %% --- Fit plane to TOP points and keep correct indices ---
    zThreshold = prctile(xyzAll(:,3), 80);
    topMask = xyzAll(:,3) > zThreshold;
    xyzTopCandidate = xyzAll(topMask, :);

    topCandidateCloud = pointCloud(xyzTopCandidate);
    [modelTop, inliersTop] = pcfitplane(topCandidateCloud, 0.0015);

    topCloud = select(topCandidateCloud, inliersTop);

    figure; pcshow(topCloud, 'VerticalAxisDir','Down'); axis equal; title('Top Surface');

    %% --- PCA-based corner detection on top surface ---
    xyzTop = topCloud.Location;
    xyzTop = reshape(xyzTop, [], 3);
    xyzTop = xyzTop(~any(isnan(xyzTop), 2), :);

    centroid = mean(xyzTop, 1);
    centered = xyzTop - centroid;

    [coeff, ~, ~] = pca(centered);

    % Ensure the 3rd axis (plane normal) points consistently
    if coeff(3,3) < 0
        coeff(:,3) = -coeff(:,3);
    end

    % Ensure axes 1 and 2 are consistently oriented
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

    %% --- Build corners in PCA frame ---
    corners_rot = [
        xmin, ymin, 0;
        xmax, ymin, 0;
        xmax, ymax, 0;
        xmin, ymax, 0
    ];

    corners_temp = corners_rot * coeff' + centroid;

    %% --- Project corners onto actual fitted plane ---
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

    cornersPlot = [corners; corners(1,:)];

    side1 = norm(corners(2,:) - corners(1,:));
    side2 = norm(corners(3,:) - corners(2,:));
    fprintf('Corner side lengths: %.4f m  x  %.4f m\n', side1, side2);
    fprintf('Corners (camera frame XYZ):\n');
    disp(corners);

    %% ========== DEFINE BLOCK FRAME ==========
    % Origin: Corner 1 (you can choose any corner: 1, 2, 3, or 4)
    block_origin = corners(1, :);
    
    % X-axis: along edge from corner 1 to corner 2
    block_x_axis = (corners(2,:) - corners(1,:)) / norm(corners(2,:) - corners(1,:));
    
    % Y-axis: along edge from corner 1 to corner 4
    block_y_axis = (corners(4,:) - corners(1,:)) / norm(corners(4,:) - corners(1,:));
    
    % Z-axis: perpendicular to top surface (pointing upward)
    % Use the plane normal from fitted plane
    block_z_axis = [A, B, C] / norm([A, B, C]);
    
    % Ensure Z points upward (positive Z direction)
    if block_z_axis(3) < 0
        block_z_axis = -block_z_axis;
    end
    
    % Construct rotation matrix: columns are the axes
    R_block = [block_x_axis', block_y_axis', block_z_axis'];
    
    % Homogeneous transformation matrix (block frame w.r.t. world/camera frame)
    T_block = eye(4);
    T_block(1:3, 1:3) = R_block;
    T_block(1:3, 4) = block_origin';
    
    fprintf('\n=== BLOCK FRAME ===\n');
    fprintf('Origin (world coords): [%.4f, %.4f, %.4f]\n', block_origin);
    fprintf('X-axis (world coords): [%.4f, %.4f, %.4f]\n', block_x_axis);
    fprintf('Y-axis (world coords): [%.4f, %.4f, %.4f]\n', block_y_axis);
    fprintf('Z-axis (world coords): [%.4f, %.4f, %.4f]\n', block_z_axis);
    fprintf('\nTransformation matrix T_block (block w.r.t. world):\n');
    disp(T_block);
    
    %% ========== DEFINE WORLD FRAME ==========
    % World frame origin at (0, 0, 0) with standard axes
    world_origin = [0, 0, 0];
    world_x_axis = [1, 0, 0];
    world_y_axis = [0, 1, 0];
    world_z_axis = [0, 0, 1];
    
    %% ========== VISUALIZATION ==========
    % Frame axis length for visualization
    axis_length = 0.03;  % 3 cm
    
    % 3D view
    figure
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(3); grid on; hold on;
    
    % Plot corners
    plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
          'r-o', 'MarkerSize', 14, 'LineWidth', 3);
    
    % Plot WORLD FRAME (at origin)
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
    
    % Plot BLOCK FRAME (at corner 1)
    quiver3(block_origin(1), block_origin(2), block_origin(3), ...
            block_x_axis(1)*axis_length, block_x_axis(2)*axis_length, block_x_axis(3)*axis_length, ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    quiver3(block_origin(1), block_origin(2), block_origin(3), ...
            block_y_axis(1)*axis_length, block_y_axis(2)*axis_length, block_y_axis(3)*axis_length, ...
            'g', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    quiver3(block_origin(1), block_origin(2), block_origin(3), ...
            block_z_axis(1)*axis_length, block_z_axis(2)*axis_length, block_z_axis(3)*axis_length, ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    text(block_origin(1) + block_x_axis(1)*axis_length, ...
         block_origin(2) + block_x_axis(2)*axis_length, ...
         block_origin(3) + block_x_axis(3)*axis_length, ...
         'X_{block}', 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(block_origin(1) + block_y_axis(1)*axis_length, ...
         block_origin(2) + block_y_axis(2)*axis_length, ...
         block_origin(3) + block_y_axis(3)*axis_length, ...
         'Y_{block}', 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    text(block_origin(1) + block_z_axis(1)*axis_length, ...
         block_origin(2) + block_z_axis(2)*axis_length, ...
         block_origin(3) + block_z_axis(3)*axis_length, ...
         'Z_{block}', 'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
    
    hold off;
    legend('Point Cloud', 'Block Corners', 'World Frame', '', '', ...
           'Block Frame', '', '', 'Location', 'best');
    title('Block Frame and World Frame (3D View)');
    
    % Top view
    figure
    pcshow(croppedCloud, 'VerticalAxisDir','Down', 'MarkerSize', 60);
    axis equal; view(2); grid on; hold on;
    
    % Plot corners
    plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), ...
          'r-o', 'MarkerSize', 14, 'LineWidth', 3);
    
    % Plot WORLD FRAME
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            axis_length, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(world_origin(1), world_origin(2), world_origin(3), ...
            0, axis_length, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    % Plot BLOCK FRAME
    quiver3(block_origin(1), block_origin(2), block_origin(3), ...
            block_x_axis(1)*axis_length, block_x_axis(2)*axis_length, block_x_axis(3)*axis_length, ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    quiver3(block_origin(1), block_origin(2), block_origin(3), ...
            block_y_axis(1)*axis_length, block_y_axis(2)*axis_length, block_y_axis(3)*axis_length, ...
            'g', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    
    text(block_origin(1) + block_x_axis(1)*axis_length, ...
         block_origin(2) + block_x_axis(2)*axis_length, ...
         block_origin(3), 'X_{block}', 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(block_origin(1) + block_y_axis(1)*axis_length, ...
         block_origin(2) + block_y_axis(2)*axis_length, ...
         block_origin(3), 'Y_{block}', 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    
    hold off;
    title('Block Frame and World Frame (Top View)');
end