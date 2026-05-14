%% =========================================================
% Task 8.2 – Milestone 2: Multi-Cube Autonomous Pick & Place
%
% System overview:
%   - 8 cubes placed randomly; ≥3 are the target colour (red).
%   - Remaining cubes are distractors (other colours).
%   - Robot autonomously identifies ALL target-colour cubes,
%     picks each one, and drops it into the paper basket.
%   - If a target cube is in an orientation the arm cannot
%     reach, the system FLAGS it (console + on-screen label)
%     instead of silently skipping or looping forever.
%   - Signals completion when all graspable cubes are done.
%
% FSM States (run once per cube, looped over all cubes):
%   INIT           – open jaw, move to safe home
%   PERCEIVE       – run perception pipeline, find all cubes
%   NEXT_TARGET    – pop next cube from the queue
%   PLAN           – compute IK; flag cube if unreachable
%   APPROACH_PICK  – spin base, then drop to hover
%   GRASP          – descend, close jaw
%   LIFT           – ascend to hover height
%   APPROACH_PLACE – spin base to basket, extend arm to hover
%   RELEASE        – descend, open jaw, retract
%   DONE           – success terminal state
%   ERROR          – hardware failure terminal state
% =========================================================

%% ---- USER CONFIGURATION  (tune to your physical setup) ----

% Basket location (mm, robot base frame)
BASKET_X   =  120;   % mm – measure your basket centre
BASKET_Y   =  120;   % mm
SLOT_SPACING = 50;   % mm between slot centres — tune to cube width + ~5 mm gap
BASKET_COLS  =  3;   % number of columns in the basket grid
% Heights (mm)
Z_HOVER    =  120;   % safe transit height
Z_GRASP    =   50;   % descend height for grasping
Z_PLACE    =   60;   % drop height over basket (safe drop, not too high)

% Gripper (mm jaw gap)
JAW_OPEN   =   34;   % fully open
JAW_CLOSED =   23;   % closed on cube – tune to cube width

% End-effector pitch
PHI        = -pi/2;  % gripper pointing straight down

% Motion
MOVE_SPEED  = 50;
PAUSE_MOVE  = 2.5;
PAUSE_GRIP  = 2.0;

% Reachability limits for the ungraspable check
% A cube is flagged ungraspable if IK fails OR if the cube's
% PCA tilt angle exceeds this threshold (cube on its side, etc.)
MAX_TILT_DEG = 35;   % degrees – if top-face normal tilts more than this, flag it

% =========================================================
%% HARDWARE INIT
% =========================================================
global arb;
% arb must already exist in workspace, e.g.:
%   arb = Arbotix('COM3');   % Windows
%   arb = Arbotix('/dev/ttyUSB0');  % Linux

fprintf('\n=====================================================\n');
fprintf('  Task 8.2 M2 – Multi-Cube Autonomous Pick & Place\n');
fprintf('=====================================================\n\n');

%% =========================================================
%  FSM – outer loop
% =========================================================
state = 'INIT';

% These will be populated in PERCEIVE
cube_queue       = {};   % cell array of structs with .position, .rotation, .label
delivered_count  = 0;
flagged_count    = 0;
current_cube     = [];
q_pick_hover     = [];
q_pick_grasp     = [];
q_place_hover    = [];
q_place_grasp    = [];
slot_index = 0;   % tracks next empty basket slot
% Figure handle for visual feedback (reused each loop)
fig = figure('Name', 'Task 8.2 – Live Perception', 'NumberTitle', 'off');

while true
    fprintf('[FSM] State: %s\n', state);

    switch state

        % --------------------------------------------------
        case 'INIT'
        % --------------------------------------------------
            fprintf('  Opening jaw and homing arm...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);

            home_dh = [0, 0, 0, 0];
            if ~setPosition(home_dh, MOVE_SPEED)
                fprintf('  WARNING: Could not reach home position.\n');
            end
            pause(PAUSE_MOVE);

            state = 'PERCEIVE';

        % --------------------------------------------------
        case 'PERCEIVE'
        % --------------------------------------------------
            fprintf('  Running perception pipeline...\n');
            [ptCloud, color_frame, intrinsics] = acquirePointCloud();
            mask = segmentObject(color_frame);
            
            % --- THE GHOST FIX: Count the exact number of blocks! ---
            % We look at the 2D mask and count the disconnected blobs
            % so the 3D clustering doesn't guess the wrong number!
            % Count 2D blobs FIRST to tell K-means exactly how many cubes exist
            cc    = bwconncomp(mask);
            props_2d = regionprops(cc, 'Area', 'Centroid');
            valid_blobs = find([props_2d.Area] > 500);
            actual_block_count = numel(valid_blobs);
            
            fprintf('  2D blobs: %d\n', actual_block_count);
            
            if actual_block_count == 0
                state = 'DONE'; continue
            end
            
            % Pass exact K to clustering — no more guessing
            [clusters, ~, K] = clusterObjects(objCloud, actual_block_count);
            % fprintf('  3D clusters surviving: %d (expected %d)\n', K, actual_block_count);
            % if isempty(clusters)
            %     fprintf('  Clustering returned nothing.\n');
            %     state = 'DONE';
            %     continue
            % end

            % Estimate pose for each cluster
            poses = estimatePose(clusters);

            % Build the cube queue
            % Build cube_queue from 2D blob centroids (not 3D K-means centroids)
            % This always gives one clean centroid per cube regardless of proximity
            
            fx   = intrinsics.FocalLength(1);
            fy   = intrinsics.FocalLength(2);
            cx_i = intrinsics.PrincipalPoint(1);
            cy_i = intrinsics.PrincipalPoint(2);
            
            xyz_all  = ptCloud.Location;
            validPts = ~any(isnan(xyz_all), 2) & xyz_all(:,3) > 0;
            xyz_all  = xyz_all(validPts, :);
            col_all  = round(xyz_all(:,1) ./ xyz_all(:,3) .* fx + cx_i);
            row_all  = round(xyz_all(:,2) ./ xyz_all(:,3) .* fy + cy_i);
            
            cube_queue = {};
            for i = 1:numel(valid_blobs)
                k  = valid_blobs(i);
                u  = round(props_2d(k).Centroid(1));   % pixel col
                v  = round(props_2d(k).Centroid(2));   % pixel row
            
                % Find 3D points within 10px of this blob's centroid
                near = abs(col_all - u) < 10 & abs(row_all - v) < 10;
            
                if sum(near) < 3
                    fprintf('  Blob %d: insufficient depth points, skipping.\n', i);
                    continue
                end
            
                pos_mm    = double(mean(xyz_all(near, :), 1) * 1000);
                pos_mm(3) = 25.0;   % fix Z to cube half-height
            
                entry.position = pos_mm;
                entry.rotation = eye(3);   % no PCA needed
                entry.label    = i;
                cube_queue{end+1} = entry;
            
                fprintf('  Cube %d → [%.1f, %.1f, %.1f] mm\n', i, pos_mm(1), pos_mm(2), pos_mm(3));
            end
            
            fprintf('  %d cube(s) queued.\n', numel(cube_queue));
            showPerceptionFigure(fig, color_frame, mask, cube_queue);
            state = 'NEXT_TARGET';

        % --------------------------------------------------
        case 'NEXT_TARGET'
        % --------------------------------------------------
            if isempty(cube_queue)
                state = 'DONE';
                continue
            end

            % Pop the first cube
            current_cube = cube_queue{1};
            cube_queue   = cube_queue(2:end);

            fprintf('  --- Next cube: Label %d  at [%.1f, %.1f, %.1f] mm ---\n', ...
                current_cube.label, ...
                current_cube.position(1), ...
                current_cube.position(2), ...
                current_cube.position(3));

            state = 'PLAN';

        % --------------------------------------------------
        case 'PLAN'
        % --------------------------------------------------
            fprintf('  Computing IK for cube %d...\n', current_cube.label);

            % --- Camera-to-robot frame transform ---
            % (same offsets tuned in Task 8.1)
            cx_raw = -current_cube.position(2);
            cy_raw = -current_cube.position(1);
            cx = cx_raw - 5;
            cy = cy_raw + 25;

            % --- Check orientation: is the cube graspable? ---
            % PCA gives us the principal axes. The 3rd column of R is the
            % axis of least variance, which approximates the cube's "up" direction.
            % If it is too far from world-Z, the cube is tilted / on its side.
            % cube_up  = current_cube.rotation(:, 3);  % 3rd PCA axis
            % world_up = [0; 0; 1];
            % tilt_deg = acosd(abs(dot(cube_up, world_up)));
            % 
            % if tilt_deg > MAX_TILT_DEG
            %     % --- UNGRASPABLE FLAG ---
            %     fprintf('  *** UNGRASPABLE: Cube %d is tilted %.1f deg (limit %.0f deg). SKIPPING. ***\n', ...
            %         current_cube.label, tilt_deg, MAX_TILT_DEG);
            %     flagPerceptionFigure(fig, current_cube.label, tilt_deg);
            %     flagged_count = flagged_count + 1;
            %     state = 'NEXT_TARGET';
            %     continue
            % end
            PHI_grasp = computeGraspAngle(current_cube.rotation);

            if isnan(PHI_grasp)
                fprintf('  *** Cube %d too tilted (>60 deg). Skipping. ***\n', current_cube.label);
                flagged_count = flagged_count + 1;
                state = 'NEXT_TARGET'; continue
            end
            % --- IK for pick ---
            current_servo = [arb.getpos(1), arb.getpos(2), ...
                             arb.getpos(3), arb.getpos(4)];
            current_dh    = servo2dh(current_servo);

            q_pick_hover = my_findSolution(cx, cy, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  *** UNGRASPABLE: Cube %d – no IK for pick-hover. Flagging. ***\n', ...
                    current_cube.label);
                flagPerceptionFigure(fig, current_cube.label, -1);
                flagged_count = flagged_count + 1;
                state = 'NEXT_TARGET';
                continue
            end

            q_pick_grasp = my_findSolution(cx, cy, Z_GRASP, PHI, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  *** UNGRASPABLE: Cube %d – no IK for pick-grasp. Flagging. ***\n', ...
                    current_cube.label);
                flagPerceptionFigure(fig, current_cube.label, -1);
                flagged_count = flagged_count + 1;
                state = 'NEXT_TARGET';
                continue
            end

            % --- IK for place ---
            % --- IK for place: compute next empty slot position ---
            [slot_x, slot_y] = getBasketSlot(slot_index, BASKET_X, BASKET_Y, ...
                                              SLOT_SPACING, BASKET_COLS);
            fprintf('  Slot #%d → [%.1f, %.1f] mm\n', slot_index, slot_x, slot_y);
            
            q_place_hover = my_findSolution(slot_x, slot_y, Z_HOVER, PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  ERROR: No IK for place-hover. Aborting.\n');
                state = 'ERROR';
                continue
            end
            
            q_place_grasp = my_findSolution(slot_x, slot_y, Z_PLACE, PHI, q_place_hover);
            if isempty(q_place_grasp)
                fprintf('  ERROR: No IK for place-grasp. Aborting.\n');
                state = 'ERROR';
                continue
            end
            % 
            % fprintf('  All waypoints planned. Tilt = %.1f deg (OK).\n', tilt_deg);
            state = 'APPROACH_PICK';

        % --------------------------------------------------
        case 'APPROACH_PICK'
        % --------------------------------------------------
            fprintf('  Turning base to face target...\n');
            
            % --- THE SWOOP FIX ---
            % 1. Turn the base (Joint 1) to match the target, but keep 
            % the shoulder, elbow, and wrist perfectly straight up (0,0,0)
            q_intermediate = [q_pick_hover(1), 0, 0, 0];
            setPosition(q_intermediate,50);
            pause(1.5); % Give the base time to spin
            
            fprintf('  Dropping to hover above cube...\n');
            % 2. Now bring the arm straight down to the hover position
            if ~setPosition(q_pick_hover,50)
                fprintf('  ERROR: Motion failed at APPROACH_PICK.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'GRASP';

        % --------------------------------------------------
        case 'GRASP'
        % --------------------------------------------------
            fprintf('  Descending to cube...\n');
            if ~setPosition(q_pick_grasp, MOVE_SPEED)
                fprintf('  ERROR: Motion failed descending to cube.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            fprintf('  Closing jaw...\n');
            positionJaw(JAW_CLOSED);
            pause(PAUSE_GRIP);

            state = 'LIFT';

        % --------------------------------------------------
        case 'LIFT'
        % --------------------------------------------------
            fprintf('  Lifting cube to hover height...\n');
            if ~setPosition(q_pick_hover, MOVE_SPEED)
                fprintf('  ERROR: Motion failed during lift.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            state = 'APPROACH_PLACE';

        % --------------------------------------------------
        case 'APPROACH_PLACE'
        % --------------------------------------------------
            fprintf('  Spinning base to face basket...\n');

            % Step 1: spin base toward basket, keep arm in lifted config
            q_spin = [q_place_hover(1), q_pick_hover(2), ...
                      q_pick_hover(3),  q_pick_hover(4)];
            setPosition(q_spin, MOVE_SPEED);
            pause(1.5);

            % Step 2: extend to basket hover
            fprintf('  Moving to hover above basket...\n');
            if ~setPosition(q_place_hover, MOVE_SPEED)
                fprintf('  ERROR: Motion failed at APPROACH_PLACE.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            state = 'RELEASE';

        % --------------------------------------------------
        case 'RELEASE'
        % --------------------------------------------------
            % -------------------------------------------------
        % --------------------------------------------------
            fprintf('  Dropping cube into basket...\n');
            if ~setPosition(q_place_grasp, MOVE_SPEED)
                fprintf('  ERROR: Motion failed lowering into basket.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            
            fprintf('  Opening jaw...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            
            % Retract above basket
            setPosition(q_place_hover, MOVE_SPEED);
            pause(PAUSE_MOVE);
            
            % --- THE FIX: UNTWIST THE ARM! ---
            fprintf('  Returning to safe home to untwist joints...\n');
            setPosition([0, 0, 0, 0], MOVE_SPEED);
            pause(PAUSE_MOVE);
            % ---------------------------------
            
            slot_index      = slot_index + 1;
            delivered_count = delivered_count + 1;
            fprintf('  *** Cube delivered! Total: %d ***\n', delivered_count);
            
            % Go pick the next one
            state = 'NEXT_TARGET';

        % --------------------------------------------------
        case 'DONE'
        % --------------------------------------------------
            fprintf('\n=====================================================\n');
            fprintf('  TASK COMPLETE\n');
            fprintf('  Cubes delivered : %d\n', delivered_count);
            fprintf('  Cubes flagged   : %d (ungraspable)\n', flagged_count);
            fprintf('=====================================================\n\n');

            % Return arm to home
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            setPosition([0, 0, 0, 0], MOVE_SPEED);
            pause(PAUSE_MOVE);
            break;

        % --------------------------------------------------
        case 'ERROR'
        % --------------------------------------------------
            fprintf('\n=====================================================\n');
            fprintf('  HARDWARE FAILURE – FSM aborted.\n');
            fprintf('  Check arm connection and IK parameters.\n');
            fprintf('=====================================================\n\n');
            break;

        otherwise
            fprintf('  Unknown state "%s" – aborting.\n', state);
            break;
    end
end


%% =========================================================
%  VISUAL FEEDBACK HELPERS
% =========================================================

function showPerceptionFigure(fig, color_frame, mask, cube_queue)
% Show the camera view + segmentation mask, with cube labels.

    if ~ishandle(fig), fig = figure(); end
    figure(fig);
    clf(fig);

    subplot(1, 2, 1);
    imshow(color_frame);
    title('Live Camera View');
    hold on;

    % Annotate each cube with its queue index
    for k = 1:numel(cube_queue)
        % We don't have 2D pixel locations here, so just print a note
        % (full pixel reprojection could be added with intrinsics)
    end

    subplot(1, 2, 2);
    imshow(mask);
    title(sprintf('Segmented Target Colour (%d cubes)', numel(cube_queue)));
    hold on;

    % Label connected components on the mask
    cc    = bwconncomp(mask);
    props = regionprops(cc, 'Centroid', 'Area');
    for k = 1:numel(props)
        if props(k).Area < 500, continue; end
        cx_px = props(k).Centroid(1);
        cy_px = props(k).Centroid(2);
        text(cx_px, cy_px, sprintf('#%d', k), ...
            'Color', 'yellow', 'FontSize', 14, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
    end

    drawnow;
end


function flagPerceptionFigure(fig, cube_label, tilt_deg)
% Overlay an UNGRASPABLE warning on the perception figure.

    if ~ishandle(fig), return; end
    figure(fig);

    % Work on the mask subplot (right panel)
    subplot(1, 2, 2);
    hold on;

    if tilt_deg >= 0
        reason = sprintf('Tilt %.0f deg', tilt_deg);
    else
        reason = 'IK unreachable';
    end

    % Place a red warning banner at the top of the subplot
    xl = xlim(); yl = ylim();
    text(mean(xl), yl(1) + 30 * cube_label, ...
        sprintf('*** CUBE #%d UNGRASPABLE (%s) ***', cube_label, reason), ...
        'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'white');

    % Also print clearly to console (already done in PLAN, but repeat here)
    fprintf('\n  [VISUAL FLAG] Cube #%d marked UNGRASPABLE on screen (%s).\n\n', ...
        cube_label, reason);

    drawnow;
end


%% =========================================================
%  LOCAL MOTION HELPERS
% =========================================================


%% =========================================================
%  PERCEPTION PIPELINE
% =========================================================

function [ptCloud, color_frame, intrinsics] = acquirePointCloud()

    pipe    = realsense.pipeline();
    profile = pipe.start();
    cleanupObj = onCleanup(@() stopPipelineSafely(pipe)); %#ok<NASGU>

    dev           = profile.get_device();
    depth_sensor  = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();

    depth_stream     = profile.get_stream(realsense.stream.depth) ...
                           .as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    for i = 1:5
        fs = pipe.wait_for_frames();
    end

    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);

    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data, ...
        [depth.get_width(), depth.get_height()]), [2 1]);

    MIN_DEPTH_M = 0.30;
    MAX_DEPTH_M = 0.70;
    depth_in_metres = depth_frame .* depth_scaling;
    depth_frame(depth_in_metres < MIN_DEPTH_M | ...
                depth_in_metres > MAX_DEPTH_M) = 0;

    color      = fs.get_color_frame();
    color_data = color.get_data();
    color_frame = permute(reshape(color_data, ...
        [3, color.get_width(), color.get_height()]), [3 2 1]);

    intrinsics = cameraIntrinsics( ...
        [depth_intrinsics.fx, depth_intrinsics.fy], ...
        [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
        size(depth_frame));

    ptCloudRaw = pcfromdepth(depth_frame, 1 / depth_scaling, ...
        intrinsics, 'ColorImage', color_frame);

    VOXEL_SIZE = 0.002;
    ptCloudDS  = pcdownsample(ptCloudRaw, 'gridAverage', VOXEL_SIZE);

    K_NEIGHBOURS = 20;
    N_SIGMA      = 2.0;
    [ptCloud, ~] = pcdenoise(ptCloudDS, ...
        'NumNeighbors', K_NEIGHBOURS, 'Threshold', N_SIGMA);
end

function stopPipelineSafely(pipe)
    try, pipe.stop(); catch, end
end


function mask = segmentObject(color_frame)
% HSV thresholding for red/target colour with morphological cleanup.

    rgb    = im2double(color_frame);
    hsvImg = rgb2hsv(rgb);
    h = hsvImg(:,:,1);
    s = hsvImg(:,:,2);
    v = hsvImg(:,:,3);

    % Red hue wraps around 0/1 – expanded range for lighting variation
    mask = (h < 0.08 | h > 0.92) & (s > 0.25) & (v > 0.2);

    SE_open  = strel('disk', 3);
    SE_close = strel('disk', 2);
    mask = imopen(mask,  SE_open);
    mask = imclose(mask, SE_close);
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end


function objCloud = maskPointCloud(ptCloud, mask, intrinsics)

    xyz = ptCloud.Location;
    rgb = ptCloud.Color;

    validPts = ~any(isnan(xyz), 2) & (xyz(:,3) > 0);
    xyz = xyz(validPts, :);
    rgb = rgb(validPts, :);

    if isempty(xyz)
        warning('maskPointCloud: no valid points.');
        objCloud = pointCloud(zeros(1,3));
        return
    end

    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1);
    cy = intrinsics.PrincipalPoint(2);

    col = round(xyz(:,1) ./ xyz(:,3) .* fx + cx);
    row = round(xyz(:,2) ./ xyz(:,3) .* fy + cy);

    [H, W] = size(mask);
    inBounds = col >= 1 & col <= W & row >= 1 & row <= H;
    col = col(inBounds);
    row = row(inBounds);
    xyz = xyz(inBounds, :);
    rgb = rgb(inBounds, :);

    linearIdx = sub2ind([H, W], row, col);
    inMask    = mask(linearIdx);

    xyzClean = xyz(inMask, :);
    rgbClean = rgb(inMask, :);

    if isempty(xyzClean)
        warning('maskPointCloud: no points in mask.');
        objCloud = pointCloud(zeros(1,3));
        return
    end

    objCloud = pointCloud(xyzClean, 'Color', rgbClean);
end


function [clusters, labels, K] = clusterObjects(ptCloud, K_override)

    MIN_CLUSTER_PTS = 30;
    K_MAX           = 8;
    KMEANS_REPS     = 5;
    KMEANS_ITER     = 200;

    xyz = ptCloud.Location;
    xyz = xyz(~any(isnan(xyz), 2), :);
    N   = size(xyz, 1);

    if N < MIN_CLUSTER_PTS
        warning('clusterObjects: cloud too small (%d pts).', N);
        clusters = {}; labels = []; K = 0;
        return
    end

    if nargin >= 2 && ~isempty(K_override)
        K = max(1, round(K_override));
    else
        K_max_actual = min(K_MAX, floor(N / MIN_CLUSTER_PTS));
        if K_max_actual < 2
            K = 1;
        else
            wcss = zeros(1, K_max_actual);
            for k = 1:K_max_actual
                [~, ~, sumd] = kmeans(xyz, k, ...
                    'Distance', 'sqeuclidean', 'Replicates', KMEANS_REPS, ...
                    'MaxIter', KMEANS_ITER, 'Display', 'off');
                wcss(k) = sum(sumd);
            end
            w  = wcss / wcss(1);
            ks = 1:K_max_actual;
            p1 = [ks(1), w(1)]; p2 = [ks(end), w(end)];
            d  = zeros(1, K_max_actual);
            for i = 1:K_max_actual
                p = [ks(i), w(i)];
                d(i) = abs(cross2d(p2-p1, p1-p)) / norm(p2-p1);
            end
            [~, K] = max(d);
        end
    end

    [labels, ~] = kmeans(xyz, K, ...
        'Distance', 'sqeuclidean', 'Replicates', KMEANS_REPS, ...
        'MaxIter', KMEANS_ITER, 'Display', 'off');

    validMask = ~any(isnan(ptCloud.Location), 2);
    rgb       = ptCloud.Color;
    rgb       = rgb(validMask, :);

    clusters = {};
    for k = 1:K
        idx = find(labels == k);
        if numel(idx) < MIN_CLUSTER_PTS, continue; end
        c = pointCloud(xyz(idx, :), 'Color', rgb(idx, :));
        clusters{end+1} = c; %#ok<AGROW>
    end

    K = numel(clusters);
end

function d = cross2d(a, b)
    d = a(1)*b(2) - a(2)*b(1);
end


function poses = estimatePose(clusters)

    poses = cell(1, length(clusters));
    for k = 1:length(clusters)
        xyz = clusters{k}.Location;
        xyz = xyz(~any(isnan(xyz), 2), :);

        if size(xyz, 1) < 3
            poses{k}.position = [NaN NaN NaN];
            poses{k}.rotation = eye(3);
            continue
        end

        poses{k}.position = mean(xyz, 1);
        [R, ~, ~]         = pca(xyz);
        poses{k}.rotation = R;
    end
end

function [sx, sy] = getBasketSlot(slot_idx, bx, by, spacing, ncols)
% Returns (x,y) centre of the next empty basket slot.
% Slots fill left-to-right, then front-to-back, centred on (bx,by).
    col = mod(slot_idx, ncols);
    row = floor(slot_idx / ncols);
    col_offset = (col - (ncols-1)/2) * spacing;
    sx = bx + col_offset;
    sy = by + row * spacing;
end

function phi = computeGraspAngle(rotation)
% Extract the gripper approach angle from PCA rotation matrix.
% rotation: 3x3 PCA matrix from estimatePose
% Returns PHI (radians) for setPosition — negative = downward approach

    % 3rd PCA axis = cube's surface normal (least variance direction)
    cube_normal = rotation(:, 3);   % [nx, ny, nz]
    world_up    = [0; 0; 1];

    % Tilt angle from vertical
    tilt_rad = acos(abs(dot(cube_normal, world_up)));

    % Clamp: if nearly flat (< 15 deg tilt) just use straight down
    % If tilted more, adjust approach angle accordingly
    MAX_TILT_RAD = deg2rad(60);   % beyond this, cube is on its side — unreachable

    if tilt_rad < deg2rad(15)
        phi = -pi/2;   % flat cube — straight down as normal
    elseif tilt_rad < MAX_TILT_RAD
        % Gripper tilts to match cube face
        phi = -(pi/2 - tilt_rad);
        fprintf('    Cube tilted %.1f deg — adjusting PHI to %.3f rad\n', ...
            rad2deg(tilt_rad), phi);
    else
        phi = NaN;   % signal: unreachable, cube too tilted
    end
end