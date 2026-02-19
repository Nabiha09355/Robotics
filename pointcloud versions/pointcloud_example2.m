function pointcloud_example2()
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

    %% --- FIX 1: Extract XYZ cleanly from croppedCloud ---
    xyzAll = croppedCloud.Location;
    xyzAll = reshape(xyzAll, [], 3);
    xyzAll = xyzAll(~any(isnan(xyzAll), 2), :);

    %% --- FIX 2: Fit plane to TOP points and keep correct indices ---
    % Find Z threshold (highest 20% = smallest Z values if Z increases away from cam,
    % OR largest Z values — check your axis. Adjust prctile accordingly.)
    zThreshold = prctile(xyzAll(:,3), 80);  % top surface = farthest Z in this setup
    topMask = xyzAll(:,3) > zThreshold;
    xyzTopCandidate = xyzAll(topMask, :);

    % Fit plane to top candidate points
    topCandidateCloud = pointCloud(xyzTopCandidate);
    [modelTop, inliersTop] = pcfitplane(topCandidateCloud, 0.0015);

    % FIX: select from topCandidateCloud (NOT croppedCloud — indices don't match!)
    topCloud = select(topCandidateCloud, inliersTop);

    figure; pcshow(topCloud, 'VerticalAxisDir','Down'); axis equal; title('Top Surface');

    %% --- FIX 3: PCA-based corner detection on top surface ---
    xyzTop = topCloud.Location;
    xyzTop = reshape(xyzTop, [], 3);
    xyzTop = xyzTop(~any(isnan(xyzTop), 2), :);

    centroid = mean(xyzTop, 1);
    centered = xyzTop - centroid;

    % PCA: columns of coeff are principal axes
    [coeff, ~, ~] = pca(centered);

    % FIX: Ensure the 3rd axis (plane normal) points consistently
    if coeff(3,3) < 0
        coeff(:,3) = -coeff(:,3);
    end

    % FIX: Also ensure axes 1 and 2 are consistently oriented (avoid flipping)
    if coeff(1,1) < 0
        coeff(:,1) = -coeff(:,1);
    end
    if coeff(2,2) < 0
        coeff(:,2) = -coeff(:,2);
    end

    % Project all top points into PCA frame
    rotated = centered * coeff;  % N x 3

    % Robust bounding box using tighter percentiles to reject outliers
    xmin = prctile(rotated(:,1), 2);
    xmax = prctile(rotated(:,1), 98);
    ymin = prctile(rotated(:,2), 2);
    ymax = prctile(rotated(:,2), 98);

    %% --- FIX 4: Build corners in PCA frame, then unrotate correctly ---
    % Z=0 in PCA frame = centroid plane; we'll project onto actual plane after
    corners_rot = [
        xmin, ymin, 0;
        xmax, ymin, 0;
        xmax, ymax, 0;
        xmin, ymax, 0
    ];

    % Unrotate: corners in camera frame (before plane projection)
    corners_temp = corners_rot * coeff' + centroid;

    %% --- FIX 5: Project corners onto actual fitted plane ---
    A = modelTop.Parameters(1);
    B = modelTop.Parameters(2);
    C = modelTop.Parameters(3);
    D = modelTop.Parameters(4);

    for k = 1:4
        x = corners_temp(k,1);
        y = corners_temp(k,2);
        % Ax + By + Cz + D = 0  =>  z = -(Ax + By + D) / C
        corners_temp(k,3) = -(A*x + B*y + D) / C;
    end
    corners = corners_temp;

    %% --- FIX 6: Close the rectangle for plotting ---
    cornersPlot = [corners; corners(1,:)];  % close the loop

    % Compute side lengths to verify aspect ratio
    side1 = norm(corners(2,:) - corners(1,:));
    side2 = norm(corners(3,:) - corners(2,:));
    fprintf('Corner side lengths: %.4f m  x  %.4f m\n', side1, side2);
    fprintf('Corners (camera frame XYZ):\n');
    disp(corners);

    %% --- Visualise ---
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
end
