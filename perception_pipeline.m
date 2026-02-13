%% =========================================================
%  PERCEPTION PIPELINE  –  Milestone 2
%  All functions in one file for easy deployment.
%  Changes vs. original:
%    - acquirePointCloud  : depth range clamp, voxel downsample,
%                           statistical outlier removal
%    - segmentObject      : morphological clean-up (open+close)
%    - maskPointCloud     : FIXED – projects flat [N×3] cloud back into
%                           pixel space via intrinsics to apply 2-D mask
%    - clusterObjects     : REPLACED pcsegdist with K-means clustering;
%                           auto-selects K via elbow method (WCSS curve);
%                           filters undersized clusters after assignment
%    - estimatePose       : initialise output cell, handle < 3 pts
%    - pipelinemain       : passes intrinsics to maskPointCloud;
%                           shows elbow plot + colour-coded cluster figure
%% =========================================================


%% ----------------------------------------------------------
function [ptCloud, color_frame, intrinsics] = acquirePointCloud()
% ACQUIREPOINTCLOUD  Capture one aligned RGBD frame from a RealSense
% camera and return a denoised, downsampled point cloud.
%
% Noise-reduction additions (new vs. original):
%   1. Depth range clamp  – discard points beyond 1.5 m (workspace limit)
%   2. Voxel grid downsample via pcdownsample – reduces density noise
%   3. Statistical outlier removal via pcdenoise – removes isolated points

    % ---- Pipeline setup ----
    pipe    = realsense.pipeline();
    profile = pipe.start();

    % ---- Device / depth parameters ----
    dev           = profile.get_device();
    depth_sensor  = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();

    depth_stream     = profile.get_stream(realsense.stream.depth) ...
                               .as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    % ---- Warm-up: discard first 5 frames ----
    for i = 1:5
        fs = pipe.wait_for_frames();
    end

    % ---- Align colour to depth ----
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);

    pipe.stop();

    % ---- Extract depth frame ----
    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute( ...
        reshape(depth_data', [depth.get_width(), depth.get_height()]), ...
        [2 1]);

    % ---- Clamp depth to workspace range ----
    % *** TUNE THESE to your actual setup ***
    % Measure the distance from camera to table surface and add ~0.15 m
    % above tallest box.  Keeping range tight = denser, cleaner cloud.
    MIN_DEPTH_M = 0.20;   % 20 cm  – ignore sensor blind-spot
    MAX_DEPTH_M = 0.90;   % 90 cm  – typical overhead robot workspace
    depth_in_metres = depth_frame .* depth_scaling;
    depth_frame(depth_in_metres < MIN_DEPTH_M | ...
                depth_in_metres > MAX_DEPTH_M) = 0;

    % ---- Extract colour frame ----
    color      = fs.get_color_frame();
    color_data = color.get_data();
    color_frame = permute( ...
        reshape(color_data', [3, color.get_width(), color.get_height()]), ...
        [3 2 1]);

    % ---- Camera intrinsics (MATLAB format) ----
    intrinsics = cameraIntrinsics( ...
        [depth_intrinsics.fx,  depth_intrinsics.fy], ...
        [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
        size(depth_frame));

    % ---- Build raw point cloud ----
    ptCloudRaw = pcfromdepth( ...
        depth_frame, ...
        1 / depth_scaling, ...
        intrinsics, ...
        ColorImage = color_frame);

    % ---- Voxel-grid downsample ----
    % 2 mm leaf keeps the cloud DENSE enough to look solid while still
    % smoothing per-pixel depth jitter.  Increase to 0.004 if slow.
    VOXEL_SIZE = 0.002;
    ptCloudDS  = pcdownsample(ptCloudRaw, 'gridAverage', VOXEL_SIZE);

    % ---- Statistical outlier removal ----
    % Looser threshold (2.0 sigma) removes only true outlier spikes
    % without punching holes in box surfaces.
    K_NEIGHBOURS = 20;
    N_SIGMA      = 2.0;
    [ptCloud, ~] = pcdenoise(ptCloudDS, ...
        'NumNeighbors', K_NEIGHBOURS, ...
        'Threshold',    N_SIGMA);
end


%% ----------------------------------------------------------
function mask = segmentObject(color_frame)
% SEGMENTOBJECT  Produce a binary mask of red objects using HSV thresholding.
%
% Noise-reduction additions:
%   - Morphological opening  (removes small false-positive speckles)
%   - Morphological closing  (fills small holes before imfill)

    rgb    = im2double(color_frame);
    hsvImg = rgb2hsv(rgb);
    h = hsvImg(:,:,1);
    s = hsvImg(:,:,2);
    v = hsvImg(:,:,3);

    % Red hue wraps around 0/1 in HSV
    mask = (h < 0.05 | h > 0.95) & (s > 0.4) & (v > 0.3);

    % ---- NEW: morphological noise suppression ----
    SE_open  = strel('disk', 3);   % remove tiny speckles
    SE_close = strel('disk', 5);   % bridge small interior gaps
    mask = imopen(mask,  SE_open);
    mask = imclose(mask, SE_close);

    % Original post-processing (kept)
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end


%% ----------------------------------------------------------
function objCloud = maskPointCloud(ptCloud, mask, intrinsics)
% MASKPOINTCLOUD  Keep only the 3-D points whose back-projection falls
% inside the 2-D segmentation mask.
%
% ROOT CAUSE OF PREVIOUS ERROR:
%   After pcdownsample / pcdenoise, ptCloud.Location is [N×3] (a flat
%   list), NOT [H×W×3].  The old code did repmat(mask,[1 1 3]) and tried
%   to index into the 3-D array, which fails with
%   "Attempt to grow array along ambiguous dimension."
%
% FIX:
%   Project every 3-D point forward through the pinhole camera model to
%   get its (col, row) pixel coordinate, then look up whether that pixel
%   is inside the mask.  No array-shape assumptions are made.
%
% INPUTS:
%   ptCloud    – pointCloud object (flat [N×3] Location after denoising)
%   mask       – logical [H×W] segmentation mask (from segmentObject)
%   intrinsics – cameraIntrinsics object (from acquirePointCloud)

    xyz = ptCloud.Location;   % [N×3]  – always flat after pcdenoise
    rgb = ptCloud.Color;      % [N×3]

    % ---- Remove points that are already NaN or at the origin ----
    validPts = ~any(isnan(xyz), 2) & (xyz(:,3) > 0);
    xyz = xyz(validPts, :);
    rgb = rgb(validPts, :);

    if isempty(xyz)
        warning('maskPointCloud: no valid points before masking.');
        objCloud = pointCloud(zeros(1,3));
        return
    end

    % ---- Project [X Y Z] → pixel [col row] via pinhole model ----
    % intrinsics.FocalLength  = [fx  fy]
    % intrinsics.PrincipalPoint = [cx  cy]   (MATLAB: [col row] = [x y])
    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1);
    cy = intrinsics.PrincipalPoint(2);

    % Perspective projection (Z is the optical-axis depth)
    col = round(xyz(:,1) ./ xyz(:,3) .* fx + cx);   % pixel column
    row = round(xyz(:,2) ./ xyz(:,3) .* fy + cy);   % pixel row

    [H, W] = size(mask);

    % ---- Keep only points that land inside the image boundary ----
    inBounds = col >= 1 & col <= W & row >= 1 & row <= H;
    col = col(inBounds);
    row = row(inBounds);
    xyz = xyz(inBounds, :);
    rgb = rgb(inBounds, :);

    % ---- Look up mask value at each projected pixel ----
    linearIdx  = sub2ind([H, W], row, col);
    inMask     = mask(linearIdx);

    xyzClean = xyz(inMask, :);
    rgbClean = rgb(inMask, :);

    if isempty(xyzClean)
        warning('maskPointCloud: no points fall inside the mask.');
        objCloud = pointCloud(zeros(1,3));
        return
    end

    objCloud = pointCloud(xyzClean, 'Color', rgbClean);
end


%% ----------------------------------------------------------
function [clusters, labels, K] = clusterObjects(ptCloud, K_override)
% CLUSTEROBJECTS  Cluster the masked point cloud using K-means on XYZ.
%
% WHY K-MEANS INSTEAD OF pcsegdist:
%   pcsegdist (Euclidean distance) requires careful threshold tuning and
%   can over-segment a single box into many fragments when surface noise
%   exceeds the threshold.  K-means assigns every point to exactly one of
%   K compact, convex clusters, which maps naturally onto "one cluster per
%   box" when the number of boxes K is known (or estimated automatically).
%
% AUTO-K SELECTION (elbow method):
%   When K_override is not supplied, the function evaluates K = 1..K_MAX
%   and picks the K after which WCSS (within-cluster sum of squares) stops
%   dropping sharply.  The elbow is detected as the point of maximum
%   curvature on the normalised WCSS curve.
%
% INPUTS:
%   ptCloud    – pointCloud object (flat [N×3], output of maskPointCloud)
%   K_override – (optional) integer; forces a specific K, skips auto-K
%
% OUTPUTS:
%   clusters   – 1×K cell array of pointCloud objects, one per cluster
%   labels     – N×1 integer vector of cluster assignments (1..K)
%   K          – number of clusters actually used

    MIN_CLUSTER_PTS = 30;   % discard clusters smaller than this
    K_MAX           = 8;    % upper bound for auto-K search
    KMEANS_REPS     = 5;    % independent restarts (best WCSS kept)
    KMEANS_ITER     = 200;  % max iterations per restart

    % ----------------------------------------------------------------
    % 0. Input validation
    % ----------------------------------------------------------------
    xyz = ptCloud.Location;   % [N×3]
    xyz = xyz(~any(isnan(xyz), 2), :);   % strip any residual NaNs

    N = size(xyz, 1);

    if N < MIN_CLUSTER_PTS
        warning('clusterObjects: cloud too small (%d pts) for K-means.', N);
        clusters = {};
        labels   = [];
        K        = 0;
        return
    end

    % ----------------------------------------------------------------
    % 1. Determine K
    % ----------------------------------------------------------------
    if nargin >= 2 && ~isempty(K_override)
        % User supplied K directly – use it
        K = max(1, round(K_override));
        fprintf('clusterObjects: using user-supplied K = %d\n', K);
    else
        % Auto-select K via elbow on WCSS
        K_max_actual = min(K_MAX, floor(N / MIN_CLUSTER_PTS));

        if K_max_actual < 2
            % Not enough points to test multiple K values
            K = 1;
        else
            wcss = zeros(1, K_max_actual);
            for k = 1:K_max_actual
                [~, ~, sumd] = kmeans(xyz, k, ...
                    'Distance',   'sqeuclidean', ...
                    'Replicates', KMEANS_REPS,   ...
                    'MaxIter',    KMEANS_ITER,   ...
                    'Display',    'off');
                wcss(k) = sum(sumd);
            end

            % Normalise WCSS to [0,1] then find elbow = max curvature
            % using the perpendicular distance from the line
            % (1, wcss_norm(1)) → (K_max, wcss_norm(K_max)).
            w  = wcss / wcss(1);          % normalise so first value = 1
            ks = 1:K_max_actual;
            % Vector from first to last point on the curve
            p1 = [ks(1),   w(1)  ];
            p2 = [ks(end), w(end)];
            d  = zeros(1, K_max_actual);
            for i = 1:K_max_actual
                p  = [ks(i), w(i)];
                d(i) = abs(cross2d(p2-p1, p1-p)) / norm(p2-p1);
            end
            [~, K] = max(d);

            fprintf('clusterObjects: auto-selected K = %d  (elbow method)\n', K);

            % Store wcss for optional plotting by pipelinemain
            assignin('base', 'kmeans_wcss',   wcss);
            assignin('base', 'kmeans_K_range', ks);
        end
    end

    % ----------------------------------------------------------------
    % 2. Final K-means with chosen K
    % ----------------------------------------------------------------
    [labels, centroids] = kmeans(xyz, K, ...
        'Distance',   'sqeuclidean', ...
        'Replicates', KMEANS_REPS,   ...
        'MaxIter',    KMEANS_ITER,   ...
        'Display',    'off');

    fprintf('clusterObjects: K-means converged with K = %d\n', K);
    fprintf('  Centroids [X Y Z] (metres):\n');
    for k = 1:K
        fprintf('    Cluster %d:  X=%.4f  Y=%.4f  Z=%.4f\n', ...
                k, centroids(k,1), centroids(k,2), centroids(k,3));
    end

    % ----------------------------------------------------------------
    % 3. Build per-cluster pointCloud objects
    % ----------------------------------------------------------------
    rgb = ptCloud.Location;   % placeholder – re-fetch real colors below
    rgb = ptCloud.Color;      % [N×3] uint8

    % Re-strip NaNs from colour in sync with xyz strip above
    validMask = ~any(isnan(ptCloud.Location), 2);
    rgb       = rgb(validMask, :);

    clusters = {};
    for k = 1:K
        idx = find(labels == k);
        if numel(idx) < MIN_CLUSTER_PTS
            warning('clusterObjects: cluster %d has only %d pts – skipping.', ...
                    k, numel(idx));
            continue
        end
        c = pointCloud(xyz(idx, :), 'Color', rgb(idx, :));
        clusters{end+1} = c; %#ok<AGROW>
    end

    if isempty(clusters)
        warning('clusterObjects: all K-means clusters below minimum size.');
    else
        fprintf('clusterObjects: %d valid clusters returned.\n', numel(clusters));
    end
end


%% ----------------------------------------------------------
function d = cross2d(a, b)
% CROSS2D  Scalar cross product magnitude for 2-D vectors a and b.
    d = a(1)*b(2) - a(2)*b(1);
end


%% ----------------------------------------------------------
function poses = estimatePose(clusters)
% ESTIMATEPOSE  Compute centroid + PCA orientation for each cluster.
%
% Change vs. original: initialise output cell explicitly; guard against
% degenerate clusters with fewer than 3 unique points (PCA requires >= 3).

    poses = cell(1, length(clusters));   % FIX: pre-initialise

    for k = 1:length(clusters)
        xyz = clusters{k}.Location;

        % Remove any residual NaNs
        xyz = xyz(~any(isnan(xyz), 2), :);

        % ---- Guard: need at least 3 points for PCA ----
        if size(xyz, 1) < 3
            warning('estimatePose: cluster %d has < 3 pts; skipping.', k);
            poses{k}.position = [NaN NaN NaN];
            poses{k}.rotation = eye(3);
            continue
        end

        t = mean(xyz, 1);

        % PCA: columns of R are principal axes (sorted by variance)
        [R, ~, ~] = pca(xyz);

        poses{k}.position = t;
        poses{k}.rotation = R;
    end
end


%% ----------------------------------------------------------
function pipelinemain()
% PIPELINEMAIN  Top-level orchestration of the perception pipeline.

    fprintf('=== Perception Pipeline Start ===\n');

    % ---- Stage 1: Acquire & denoise point cloud ----
    fprintf('Stage 1: Acquiring point cloud...\n');
    [ptCloud, color_frame, intrinsics] = acquirePointCloud();

    figure;
    pcshow(ptCloud, 'MarkerSize', 6);
    title('Stage 1: Full Scene Point Cloud (denoised)', 'Color','w');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis tight; grid on;

    % ---- Stage 2: Colour segmentation ----
    fprintf('Stage 2: Segmenting object...\n');
    mask = segmentObject(color_frame);

    figure;
    imshow(mask);
    title('Stage 2: 2-D Object Segmentation Mask');

    % ---- Stage 3: Mask point cloud ----
    fprintf('Stage 3: Masking point cloud...\n');
    objCloud = maskPointCloud(ptCloud, mask, intrinsics);

    figure;
    pcshow(objCloud, 'MarkerSize', 8);
    title('Stage 3: Masked Object Point Cloud', 'Color','w');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis tight; grid on;

    % ---- Stage 4: K-means Clustering ----
    fprintf('Stage 4: Clustering objects with K-means...\n');
    % Pass a second argument to force a specific K, e.g. clusterObjects(objCloud, 3)
    % Leave empty / omit to auto-select K via the elbow method.
    [clusters, labels, K] = clusterObjects(objCloud);

    if isempty(clusters)
        warning('pipelinemain: no clusters found – check segmentation.');
        return
    end

    % ---- Elbow plot (only when auto-K was used) ----
    if evalin('base', 'exist(''kmeans_wcss'',''var'')')
        wcss   = evalin('base', 'kmeans_wcss');
        ks     = evalin('base', 'kmeans_K_range');
        figure;
        plot(ks, wcss, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
        xline(K, 'r--', sprintf('K = %d', K), 'LabelVerticalAlignment', 'bottom');
        title('Stage 4a: K-means Elbow Plot (WCSS vs K)');
        xlabel('Number of Clusters K');
        ylabel('Within-Cluster Sum of Squares');
        grid on;
    end

    % ---- Colour-coded cluster plot ----
    clusterColors = lines(numel(clusters));
    figure;
    set(gcf, 'Color', 'white');
    hold on;
    for k = 1:numel(clusters)
        pts = clusters{k}.Location;
        scatter3(pts(:,1), pts(:,2), pts(:,3), 10, ...
                 repmat(clusterColors(k,:), size(pts,1), 1), 'filled');
    end
    hold off;
    title(sprintf('Stage 4b: K-means Clusters (K = %d)', K));
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend(arrayfun(@(k) sprintf('Cluster %d', k), ...
           1:numel(clusters), 'UniformOutput', false), ...
           'Location', 'best');
    view(3); axis tight equal; grid on;

    % ---- Individual cluster figures ----
    for k = 1:numel(clusters)
        figure;
        pcshow(clusters{k}, 'MarkerSize', 10);
        title(sprintf('Stage 4c: Cluster %d  (%d pts)', k, clusters{k}.Count), ...
              'Color', 'w');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        axis tight; grid on;
    end

    % ---- Stage 5: Pose estimation ----
    fprintf('Stage 5: Estimating poses...\n');
    poses = estimatePose(clusters);

    for k = 1:length(poses)
        fprintf('\n===== Object %d Pose =====\n', k);
        fprintf('Position [x y z] (metres):\n');
        disp(poses{k}.position);
        fprintf('Orientation (Rotation Matrix – PCA axes):\n');
        disp(poses{k}.rotation);
    end

    disp('=== Pipeline execution complete. ===');
end
