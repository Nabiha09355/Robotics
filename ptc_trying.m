function ptc_trying()
    % 1. Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    profile = pipe.start();

    %% 2. Acquire device parameters 
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    %% 3. Warm-up and Capture Snapshot
    for i = 1:20 % Standard warm-up
        fs = pipe.wait_for_frames();
    end

    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    pipe.stop(); % Stop streaming immediately after snapshot

    % Get data
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[depth.get_width(), depth.get_height()]),[2 1]);

    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3, color.get_width(), color.get_height()]),[3 2 1]);

    %% 4. Create Point Cloud
    intrinsicsObj = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
                                     [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
                                     size(depth_frame));
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsicsObj, ColorImage=color_frame);

    %% 5. ROI Crop & Denoise
    roi = [-0.1 0.1 0 0.10 0 0.385]; % Adjust based on your setup
    indices = findPointsInROI(ptCloud, roi);
    croppedCloud = select(ptCloud, indices);
    croppedCloud = pcdenoise(croppedCloud, 'NumNeighbors', 20, 'Threshold', 1);

    %% 6. Extract Top Surface & Fit Plane
    xyzAll = reshape(croppedCloud.Location, [], 3);
    xyzAll = xyzAll(~any(isnan(xyzAll), 2), :);

    zThreshold = prctile(xyzAll(:,3), 80);  
    topMask = xyzAll(:,3) > zThreshold;
    xyzTopCandidate = xyzAll(topMask, :);

    topCandidateCloud = pointCloud(xyzTopCandidate);
    [modelTop, inliersTop] = pcfitplane(topCandidateCloud, 0.0015);
    topCloud = select(topCandidateCloud, inliersTop);

    %% 7. PCA and Corner Detection
    xyzTop = reshape(topCloud.Location, [], 3);
    xyzTop = xyzTop(~any(isnan(xyzTop), 2), :);

    % Find Centroid
    minXYZ = min(xyzTop, [], 1);
    maxXYZ = max(xyzTop, [], 1);
    centroid = (minXYZ + maxXYZ) / 2; 
    
    % Run PCA on centered points
    centeredPoints = xyzTop - centroid;
    [coeff, ~, ~] = pca(centeredPoints);

    % Orientation Fixes
    if coeff(3,3) < 0, coeff(:,3) = -coeff(:,3); end
    if coeff(1,1) < 0, coeff(:,1) = -coeff(:,1); end
    if coeff(2,2) < 0, coeff(:,2) = -coeff(:,2); end

    % Project to PCA space to find boundaries
    rotated = centeredPoints * coeff;
    xmin = prctile(rotated(:,1), 2); xmax = prctile(rotated(:,1), 98);
    ymin = prctile(rotated(:,2), 2); ymax = prctile(rotated(:,2), 98);

    % Corners back to camera frame
    corners_rot = [xmin, ymin, 0; xmax, ymin, 0; xmax, ymax, 0; xmin, ymax, 0];
    corners_temp = corners_rot * coeff' + centroid;

    % Snap corners to fitted plane
    A = modelTop.Parameters(1); B = modelTop.Parameters(2);
    C = modelTop.Parameters(3); D = modelTop.Parameters(4);
    for k = 1:4
        x = corners_temp(k,1); y = corners_temp(k,2);
        corners_temp(k,3) = -(A*x + B*y + D) / C;
    end
    cornersPlot = [corners_temp; corners_temp(1,:)]; 

    %% 8. Calculate Object Frame Vectors
    v1 = corners_temp(2,:) - corners_temp(1,:);
    v2 = corners_temp(3,:) - corners_temp(2,:);
    
    if norm(v1) < norm(v2)
        xAxis = v1 / norm(v1);
    else
        xAxis = v2 / norm(v2);
    end

    zAxis = [A, B, C] / norm([A, B, C]);
    if zAxis(3) < 0, zAxis = -zAxis; end
    yAxis = cross(zAxis, xAxis);
    yAxis = yAxis / norm(yAxis);

    %% 9. Project 3D Points to 2D RGB Pixels
    axisLen = 0.04; % 4cm axes
    tipX_3d = centroid + xAxis * axisLen;
    tipY_3d = centroid + yAxis * axisLen;
    tipZ_3d = centroid + zAxis * axisLen;
    
    pts3d = [centroid; tipX_3d; tipY_3d; tipZ_3d; corners_temp]; 

    % Projection Logic
    pts2d = zeros(size(pts3d, 1), 2);
    for i = 1:size(pts3d, 1)
        pts2d(i, 1) = (pts3d(i, 1) * depth_intrinsics.fx / pts3d(i, 3)) + depth_intrinsics.ppx;
        pts2d(i, 2) = (pts3d(i, 2) * depth_intrinsics.fy / pts3d(i, 3)) + depth_intrinsics.ppy;
    end

    c2d = pts2d(1,:); tx2d = pts2d(2,:); ty2d = pts2d(3,:); tz2d = pts2d(4,:);
    crns2d = [pts2d(5:8,:); pts2d(5,:)]; % Close corner loop

    %% 10. Visualization
    % --- FIGURE 1: Point Cloud Overlay ---
    figure('Name', '3D Point Cloud Frame');
    pcshow(croppedCloud, 'VerticalAxisDir','Down'); hold on;
    plot3(cornersPlot(:,1), cornersPlot(:,2), cornersPlot(:,3), 'r-', 'LineWidth', 2);
    qScale = 0.04;
    quiver3(centroid(1), centroid(2), centroid(3), xAxis(1), xAxis(2), xAxis(3), qScale, 'r', 'LineWidth', 3);
    quiver3(centroid(1), centroid(2), centroid(3), yAxis(1), yAxis(2), yAxis(3), qScale, 'g', 'LineWidth', 3);
    quiver3(centroid(1), centroid(2), centroid(3), zAxis(1), zAxis(2), zAxis(3), qScale, 'b', 'LineWidth', 3);
    title('3D Block Frame');

    % --- FIGURE 2: RGB Dual Overlay ---
    figure('Name', 'RGB Image Dual Frame');
    imshow(color_frame); hold on;
    
    % Camera/World Frame (Center)
    [imgH, imgW, ~] = size(color_frame);
    camO = [imgW/2, imgH/2];
    line([camO(1) camO(1)+50], [camO(2) camO(2)], 'Color', 'r', 'LineWidth', 2);
    line([camO(1) camO(1)], [camO(2) camO(2)+50], 'Color', 'g', 'LineWidth', 2);
    plot(camO(1), camO(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    text(camO(1)+5, camO(2)-15, 'World Frame', 'Color', 'y', 'FontWeight', 'bold');

    % Block Frame (Projected)
    plot(crns2d(:,1), crns2d(:,2), 'r-', 'LineWidth', 2);
    line([c2d(1) tx2d(1)], [c2d(2) tx2d(2)], 'Color', 'r', 'LineWidth', 3);
    line([c2d(1) ty2d(1)], [c2d(2) ty2d(2)], 'Color', 'g', 'LineWidth', 3);
    line([c2d(1) tz2d(1)], [c2d(2) tz2d(2)], 'Color', 'b', 'LineWidth', 3);
    
    % Axis Labels (White Font)
    text(tx2d(1), tx2d(2), 'X', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 12);
    text(ty2d(1), ty2d(2), 'Y', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 12);
    text(tz2d(1), tz2d(2), 'Z', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 12);
    
    title('Final Dual Overlay: Camera Center vs Block Surface');
end