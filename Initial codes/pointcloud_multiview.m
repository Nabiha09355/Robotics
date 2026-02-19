function pointcloud_multiview()
clc; close all;

numViews = 3;
blockClouds = cell(1, numViews);

%% ===============================
%  CAPTURE BLOCK CLOUDS (MULTI-VIEW)
% ===============================
for i = 1:numViews
    fprintf('Move ONLY the SAME block to pose %d and press a key\n', i);
    pause;

    % ---- Single-view pipeline up to block extraction ----
    [ptCloud, color_frame] = acquirePointCloud();

    mask = segmentObject(color_frame);

    objCloud = maskPointCloud(ptCloud, mask);

    % ---- CLEANING (CRITICAL FOR ICP) ----
    xyz = objCloud.Location;
    rgb = objCloud.Color;

    validIdx = ~any(isnan(xyz), 2);
    xyz = xyz(validIdx, :);
    rgb = rgb(validIdx, :);

    cleanCloud = pointCloud(xyz, 'Color', rgb);

    % ---- DOWNSAMPLING (ICP STABILITY) ----
    cleanCloud = pcdownsample(cleanCloud, 'gridAverage', 0.005);

    % ---- VALIDATION ----
    if cleanCloud.Count < 300
        error('View %d failed: not enough points after segmentation.', i);
    end

    blockClouds{i} = cleanCloud;

    % ---- DEBUG VISUALIZATION ----
    figure;
    pcshow(cleanCloud);
    title(sprintf('Block Cloud - View %d (%d points)', ...
        i, cleanCloud.Count));
end

%% ===============================
%  MULTI-VIEW STITCHING (ICP)
% ===============================
fusedBlockCloud = blockClouds{1};
refCloud = fusedBlockCloud;

for i = 2:numViews
    tform = pcregistericp( ...
        blockClouds{i}, ...
        refCloud, ...
        'Metric','pointToPlane', ...
        'MaxIterations',50, ...
        'Tolerance',[0.001 0.005]);

    alignedCloud = pctransform(blockClouds{i}, tform);

    fusedBlockCloud = pcmerge(fusedBlockCloud, alignedCloud, 0.005);

    refCloud = fusedBlockCloud;
end

%% ===============================
%  VISUALIZE FUSED BLOCK CLOUD
% ===============================
figure;
pcshow(fusedBlockCloud);
title('Fused Block Point Cloud (Multi-View)');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% ===============================
%  POSE ESTIMATION ON FUSED CLOUD
% ===============================
poses = estimatePose({fusedBlockCloud});

disp('Final pose from fused block cloud:');
disp(poses{1});

end
