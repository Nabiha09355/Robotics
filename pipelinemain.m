function pipelinemain()

    % 1. Acquire data
    [ptCloud, color_frame] = acquirePointCloud();

    figure;
pcshow(ptCloud);
title('Stage 1: Full Scene Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');


    % 2. Segment object in image space
    mask = segmentObject(color_frame);
    figure;
imshow(mask);
title('Stage 2: 2D Object Segmentation Mask');


    % 3. Mask point cloud
    objCloud = maskPointCloud(ptCloud, mask);
    figure;
pcshow(objCloud);
title('Stage 3: Masked Object Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');


    % 4. Cluster objects
    clusters = clusterObjects(objCloud);
    for k = 1:length(clusters)
    figure;
    pcshow(clusters{k});
    title(sprintf('Stage 4: Cluster %d', k));
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

    % 5. Estimate pose
    poses = estimatePose(clusters);
    for k = 1:length(poses)
    fprintf('\n===== Object %d Pose =====\n', k);
    fprintf('Position [x y z]:\n');
    disp(poses{k}.position);
    fprintf('Orientation (Rotation Matrix):\n');
    disp(poses{k}.rotation);
end

disp('Pipeline execution complete.');


end
