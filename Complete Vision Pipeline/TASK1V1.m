
% System overview
%   1. Perception pipeline  → finds cube position (x,y,z) in mm
%   2. FSM                  → drives the robot through every
%                             phase of the pick-and-place task
%   3. Motion pipeline      → IK + setPosition + positionJaw
%
% FSM States
%   INIT          – open jaw, move to safe home position
%   PERCEIVE      – run perception pipeline to locate cube
%   PLAN          – compute all four IK waypoints
%   APPROACH_PICK – move to hover above cube
%   GRASP         – descend, close jaw
%   LIFT          – ascend back to hover height
%   APPROACH_PLACE– move to hover above basket
%   RELEASE       – descend, open jaw, retract
%   DONE          – success terminal state
%   ERROR         – failure terminal state (no IK / comm fault)
%
% =========================================================
%% ---- USER CONFIGURATION  (tune to your physical setup) ----
% ---- Basket location (mm, robot base frame) ----------------
% Place your paper basket somewhere in the workspace and
% measure its centre position with a ruler.
BASKET_X   =   120;      % mm  – change to your basket's X
BASKET_Y   = 120;      % mm  – change to your basket's Y
% ---- Heights (mm) -------------------------------------------
Z_HOVER    = 120;      % safe transit height above table
Z_GRASP    =  50;      % height at which jaw closes on cube
                       % (tune: ~half cube height above table)
Z_PLACE    =  60;      % height at which jaw opens over basket
% ---- Gripper widths (mm jaw gap) ----------------------------
JAW_OPEN   =  34.4;      % fully open  (max ~34.59 mm)
JAW_CLOSED =  23;      % closed on cube – tune to cube width
% ---- End-effector pitch during manipulation -----------------
PHI        = -pi/2;    % gripper pointing straight down
% ---- Motion speed -------------------------------------------
MOVE_SPEED = 50;       % setpos speed argument
% ---- Pauses (seconds) ---------------------------------------
PAUSE_MOVE  = 2.5;     % wait after an arm motion
PAUSE_GRIP  = 2.0;     % wait after a gripper command
% =========================================================
%% INITIALISE HARDWARE
% =========================================================
global arb;
% arb must already be set up in the workspace, e.g.:
%   arb = Arbotix('com3');   % Windows
%   arb = Arbotix('/dev/ttyUSB0');  % Linux
fprintf('\n============================================\n');
fprintf('  Task 8.2 – Autonomous Pick and Place\n');
fprintf('============================================\n\n');
%% =========================================================
%  FSM  –  main loop
% =========================================================
state        = 'INIT';
cube_pos     = [];          % [x, y, z] set during PERCEIVE
q_pick_hover  = [];
q_pick_grasp  = [];
q_place_hover = [];
q_place_grasp = [];
while true
    fprintf('[FSM] Entering state: %s\n', state);
    switch state
        % --------------------------------------------------
        case 'INIT'
        % --------------------------------------------------
            fprintf('  Opening jaw and moving to safe home...\n');
            % Open jaw first so we never drop anything
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            % Home position: straight up, out of the way
            home_dh = [0, 0, 0, 0];
            if ~setPosition(home_dh, 50)
                fprintf('  WARNING: Could not reach home. Continuing.\n');
            end
            pause(PAUSE_MOVE);
            state = 'PERCEIVE';
        % --------------------------------------------------
        case 'PERCEIVE'
        % --------------------------------------------------
            fprintf('  Running perception pipeline...\n');
            % -- Stage 1: acquire point cloud -----------------
            [ptCloud, color_frame, intrinsics] = acquirePointCloud();
            % -- Stage 2: colour segmentation ----------------
            mask = segmentObject(color_frame);
            % -- Stage 3: mask the point cloud ---------------
            objCloud = maskPointCloud(ptCloud, mask, intrinsics);
            if objCloud.Count == 0
                fprintf('  ERROR: No red object detected.\n');
                state = 'ERROR';
                continue
            end
            % -- Stage 4: cluster (force K=1 for single cube)-
            [clusters, ~, K] = clusterObjects(objCloud, 1);
            if isempty(clusters)
                fprintf('  ERROR: Clustering returned no object.\n');
                state = 'ERROR';
                continue
            end
            % -- Stage 5: estimate pose ----------------------
            poses = estimatePose(clusters);
            if isempty(poses) || any(isnan(poses{1}.position))
                fprintf('  ERROR: Pose estimation failed.\n');
                state = 'ERROR';
                continue
            end
            % Perception pipeline works in metres – convert to mm
           % Perception pipeline works in metres – convert to mm
            % We wrap it in double() to stop the checkCollision function from crashing
            cube_pos = double(poses{1}.position * 1000);  
            
            % --- CAMERA TO ROBOT FRAME CORRECTION ---
            % The camera's Z is distance from the lens, which the robot doesn't understand.
            % Since the cube is resting on the table, we overwrite Z with the 
            % physical height of the center of the cube (e.g., 25 mm).
            cube_pos(3) = 25.0; 
            
            fprintf('  Cube detected at: X=%.1f  Y=%.1f  Z=%.1f mm\n', ...
                cube_pos(1), cube_pos(2), cube_pos(3));
            
            % -----> PASTE THE VISUAL CONFIRMATION BLOCK RIGHT HERE <-----
            
       
            fprintf('  Cube detected at: X=%.1f  Y=%.1f  Z=%.1f mm\n', ...
                cube_pos(1), cube_pos(2), cube_pos(3));
            % --- VISUAL CONFIRMATION WINDOW ---
            % Use figure(1) so it updates the same window every loop 
            % instead of opening 100 new windows and crashing your laptop!
            figure(1); 
            
            % Left side: The raw color camera feed
            subplot(1,2,1); 
            imshow(color_frame); 
            title('Live Camera View');
            
            % Right side: The isolated cube
            subplot(1,2,2); 
            imshow(mask); 
            title('Detected Target (Mask)');
            
            % Force MATLAB to draw the image immediately before moving the arm
            drawnow; 
            % ----------------------------------
            state = 'PLAN';
        % --------------------------------------------------
        case 'PLAN'
        % --------------------------------------------------
            fprintf('  Computing IK for all waypoints...\n');
            % cx = cube_pos(1);
            % cy = -cube_pos(2); nabeeha
            %neva
            % cx = cube_pos(1);
            % cy = -cube_pos(2); 
            % cx = cx - 88; 
            % cy = cy + 53;
            
            %hijab
            % cx = -cube_pos(1);  
            % cy = -cube_pos(2);  
            % cx = cx + 88+95; 
            % cy = cy - 53-265;
            
            %khadeer
            cx = -cube_pos(2);
            cy = -cube_pos(1); 
            cx = cx -5; 
            cy = cy + 25;
            % cx = -cube_pos(1);
            % cy = cube_pos(2); 
            % cx = cx - 10-38; 
            % cy = cy - 15;
            % In PLAN state — Code 1:
            [PHI_grasp, graspable] = computeGraspAngle(poses{1}.rotation, cx, cy);
            
            if ~graspable || isnan(PHI_grasp)
                fprintf('  ERROR: Cube pose is ungraspable (tilted or long-side facing).\n');
                state = 'ERROR'; continue
            end
            
        
            
            current_servo = [arb.getpos(1), arb.getpos(2), ...
                             arb.getpos(3), arb.getpos(4)];
            current_dh    = servo2dh(current_servo);
        
            % Use PHI_grasp instead of hardcoded PHI everywhere:
            q_pick_hover = my_findSolution(cx, cy, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  ERROR: No IK for pick-hover.\n');
                state = 'ERROR'; continue
            end
            
            q_pick_grasp = my_findSolution(cx, cy, Z_GRASP, PHI_grasp, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  ERROR: No IK for pick-grasp (cube may be tilted beyond arm limits).\n');
                state = 'ERROR'; continue
            end
            % ---- Place waypoints ---------------------------
            q_place_hover = my_findSolution(BASKET_X, BASKET_Y, Z_HOVER, ...
                                         PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  ERROR: No IK for place-hover.\n');
                state = 'ERROR'; continue
            end
            q_place_grasp = my_findSolution(BASKET_X, BASKET_Y, Z_PLACE, ...
                                         PHI, q_place_hover);
            if isempty(q_place_grasp)
                fprintf('  ERROR: No IK for place-grasp.\n');
                state = 'ERROR'; continue
            end
            fprintf('  All waypoints planned successfully.\n');
            state = 'APPROACH_PICK';
        % --------------------------------------------------
        % case 'APPROACH_PICK'
        % % --------------------------------------------------
        %     fprintf('  Moving to hover above cube...\n');
        % 
        %     if ~setPosition(q_pick_hover)
        %         fprintf('  ERROR: Motion failed at APPROACH_PICK.\n');
        %         state = 'ERROR'; continue
        %     end
        %     pause(PAUSE_MOVE);
        % 
        %     state = 'GRASP';
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
            if ~setPosition(q_pick_grasp,50)
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
            if ~setPosition(q_pick_hover,50)
                fprintf('  ERROR: Motion failed during lift.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'APPROACH_PLACE';
        % --------------------------------------------------
        % case 'APPROACH_PLACE'
        % % --------------------------------------------------
        %     fprintf('  Moving to hover above basket...\n');
        % 
        %     if ~setPosition(q_place_hover)
        %         fprintf('  ERROR: Motion failed at APPROACH_PLACE.\n');
        %         state = 'ERROR'; continue
        %     end
        %     pause(PAUSE_MOVE);
        % 
        %     state = 'RELEASE';
        % --------------------------------------------------
        % case 'APPROACH_PLACE'
        % % --------------------------------------------------
        %     fprintf('  Turning base to face the basket...\n');
        % 
        %     % --- THE SWOOP FIX (PART 2) ---
        %     % 1. Spin the base (Joint 1) to match the basket's angle, but keep 
        %     % the shoulder, elbow, and wrist locked in their current 'lifted' 
        %     % hover positions so the block stays safely high in the air.
        %     q_safe_spin = [q_place_hover(1), q_pick_hover(2), q_pick_hover(3), q_pick_hover(4)];
        %     setPosition(q_safe_spin);
        %     pause(1.5); % Give the base time to spin
        % 
        %     fprintf('  Moving to hover above basket...\n');
        %     % 2. Now extend the arm to the final hover position over the basket
        %     if ~setPosition(q_place_hover)
        %         fprintf('  ERROR: Motion failed at APPROACH_PLACE.\n');
        %         state = 'ERROR'; continue
        %     end
        %     pause(PAUSE_MOVE);
        %     state = 'RELEASE';
        % --------------------------------------------------
        case 'APPROACH_PLACE'
        % --------------------------------------------------
            fprintf('  Turning base to face the basket...\n');
            
            % --- THE SWOOP FIX (PART 2) ---
            % 1. Spin the base (Joint 1) to match the basket's angle, but keep 
            % the shoulder, elbow, and wrist locked in their current 'lifted' 
            % hover positions so the block stays safely high in the air.
            q_intermediate = [q_place_hover(1), q_pick_hover(2), q_pick_hover(3), q_pick_hover(4)];
            setPosition(q_intermediate,50);
            pause(1.5); % Give the base time to spin
            
            fprintf('  Moving to hover above basket...\n');
            % 2. Now extend the arm to the final hover position over the basket
            if ~setPosition(q_place_hover,50)
                fprintf('  ERROR: Motion failed at APPROACH_PLACE.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'RELEASE';
        % --------------------------------------------------
        case 'RELEASE'
        % --------------------------------------------------
            fprintf('  Lowering cube into basket...\n');
            if ~setPosition(q_place_grasp,50)
                fprintf('  ERROR: Motion failed lowering into basket.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            fprintf('  Opening jaw – releasing cube...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            % Retract back to hover so we clear the basket rim
            setPosition(q_place_hover,50);
            pause(PAUSE_MOVE);
            state = 'DONE';
        % --------------------------------------------------
        case 'DONE'
        % --------------------------------------------------
            fprintf('\n============================================\n');
            fprintf('  SUCCESS: Cube placed in basket!\n');
            fprintf('============================================\n\n');
            break;
        % --------------------------------------------------
        case 'ERROR'
        % --------------------------------------------------
            fprintf('\n============================================\n');
            fprintf('  FAILURE: FSM reached ERROR state.\n');
            fprintf('  Check perception and IK parameters.\n');
            fprintf('============================================\n\n');
            break;
        otherwise
            fprintf('  Unknown state: %s – aborting.\n', state);
            break;
    end
end
%% =========================================================
%  PERCEPTION PIPELINE FUNCTIONS
%  (copied / condensed from fullpipeline.pdf – Milestone 2)
% =========================================================
function [ptCloud, color_frame, intrinsics] = acquirePointCloud()
% Capture one aligned RGBD frame from the RealSense camera,
% apply depth clamping, voxel downsampling and statistical
% outlier removal.
    
    pipe = realsense.pipeline();
    profile = pipe.start();
    
    % --- THE FIX: Create a cleanup object to ensure pipe.stop() is always called ---
    % Even if the function crashes or is stopped by Ctrl+C, this will execute.
    cleanupObj = onCleanup(@() stopPipelineSafely(pipe));
    
    dev           = profile.get_device();
    depth_sensor  = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    
    depth_stream   = profile.get_stream(realsense.stream.depth) ...
                        .as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();
    % Warm-up: discard first 5 frames
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    % Align colour to depth
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % We do not need pipe.stop() here anymore, cleanupObj handles it.
    % Extract depth frame
    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data, ...
        [depth.get_width(), depth.get_height()]), [2 1]);
    % Clamp depth to workspace range (metres)
    % Look for anything between 30cm and 70cm away from the lens
MIN_DEPTH_M = 0.30; 
MAX_DEPTH_M = 0.70;
    depth_in_metres = depth_frame .* depth_scaling;
    depth_frame(depth_in_metres < MIN_DEPTH_M | ...
                depth_in_metres > MAX_DEPTH_M) = 0;
    % Extract colour frame
    color      = fs.get_color_frame();
    color_data = color.get_data();
    color_frame = permute(reshape(color_data, ...
        [3, color.get_width(), color.get_height()]), [3 2 1]);
    % Camera intrinsics (MATLAB format)
    intrinsics = cameraIntrinsics( ...
        [depth_intrinsics.fx, depth_intrinsics.fy], ...
        [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
        size(depth_frame));
    % Build raw point cloud
    ptCloudRaw = pcfromdepth(depth_frame, 1 / depth_scaling, ...
        intrinsics, 'ColorImage', color_frame);
    % Voxel-grid downsample  (2 mm leaf)
    VOXEL_SIZE = 0.002;
    ptCloudDS  = pcdownsample(ptCloudRaw, 'gridAverage', VOXEL_SIZE);
    % Statistical outlier removal
    K_NEIGHBOURS = 20;
    N_SIGMA      = 2.0;
    [ptCloud, ~] = pcdenoise(ptCloudDS, ...
        'NumNeighbors', K_NEIGHBOURS, 'Threshold', N_SIGMA);
end
% Helper function for safe cleanup
function stopPipelineSafely(pipe)
    try
        pipe.stop();
    catch
        % Suppress errors if the pipe was already stopped
    end
end
% ---------------------------------------------------------
function mask = segmentObject(color_frame)
% Produce a binary mask for red objects using HSV thresholding
% plus morphological noise suppression.
    rgb     = im2double(color_frame);
    hsvImg  = rgb2hsv(rgb);
    h = hsvImg(:,:,1);
    s = hsvImg(:,:,2);
    v = hsvImg(:,:,3);
    % Red hue wraps around 0/1 in HSV
    % Expanded range for red/orange hues, lower saturation/brightness limits
    mask = (h < 0.08 | h > 0.92) & (s > 0.25) & (v > 0.2);
    % Morphological noise suppression
    SE_open  = strel('disk', 3);
    SE_close = strel('disk', 5);
    mask = imopen(mask,  SE_open);
    mask = imclose(mask, SE_close);
    % Fill holes and remove tiny blobs
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end
% ---------------------------------------------------------
function objCloud = maskPointCloud(ptCloud, mask, intrinsics)
% Keep only the 3-D points whose back-projection falls inside
% the 2-D segmentation mask.
    xyz = ptCloud.Location;   % [N×3] after pcdenoise
    rgb = ptCloud.Color;
    % Remove NaN / origin points
    validPts  = ~any(isnan(xyz), 2) & (xyz(:,3) > 0);
    xyz = xyz(validPts, :);
    rgb = rgb(validPts, :);
    if isempty(xyz)
        warning('maskPointCloud: no valid points before masking.');
        objCloud = pointCloud(zeros(1,3));
        return
    end
    % Project 3-D points → pixel via pinhole model
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
        warning('maskPointCloud: no points fall inside the mask.');
        objCloud = pointCloud(zeros(1,3));
        return
    end
    objCloud = pointCloud(xyzClean, 'Color', rgbClean);
end
% ---------------------------------------------------------
function [clusters, labels, K] = clusterObjects(ptCloud, K_override)
% Cluster the masked point cloud using K-means on XYZ.
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
    % Determine K
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
                    'Distance',   'sqeuclidean', ...
                    'Replicates', KMEANS_REPS, ...
                    'MaxIter',    KMEANS_ITER, ...
                    'Display',    'off');
                wcss(k) = sum(sumd);
            end
            w  = wcss / wcss(1);
            ks = 1:K_max_actual;
            p1 = [ks(1),   w(1)  ];
            p2 = [ks(end), w(end)];
            d  = zeros(1, K_max_actual);
            for i = 1:K_max_actual
                p    = [ks(i), w(i)];
                d(i) = abs(cross2d(p2-p1, p1-p)) / norm(p2-p1);
            end
            [~, K] = max(d);
        end
    end
    [labels, ~] = kmeans(xyz, K, ...
        'Distance',   'sqeuclidean', ...
        'Replicates', KMEANS_REPS, ...
        'MaxIter',    KMEANS_ITER, ...
        'Display',    'off');
    validMask = ~any(isnan(ptCloud.Location), 2);
    rgb       = ptCloud.Color;
    rgb       = rgb(validMask, :);
    clusters = {};
    for k = 1:K
        idx = find(labels == k);
        if numel(idx) < MIN_CLUSTER_PTS
            continue
        end
        c = pointCloud(xyz(idx, :), 'Color', rgb(idx, :));
        clusters{end+1} = c; %#ok<AGROW>
    end
end
% ---------------------------------------------------------
function d = cross2d(a, b)
    d = a(1)*b(2) - a(2)*b(1);
end
% ---------------------------------------------------------
function poses = estimatePose(clusters)
% Compute centroid + PCA orientation for each cluster.
    poses = cell(1, length(clusters));
    for k = 1:length(clusters)
        xyz = clusters{k}.Location;
        xyz = xyz(~any(isnan(xyz), 2), :);
        if size(xyz, 1) < 3
            poses{k}.position = [NaN NaN NaN];
            poses{k}.rotation = eye(3);
            continue
        end
        t = mean(xyz, 1);
        [R, ~, ~] = pca(xyz);
        poses{k}.position = t;
        poses{k}.rotation = R;
    end
end 

function [phi, graspable] = computeGraspAngle(rotation, cx, cy)
% Returns PHI and a graspable flag.
% rotation: 3x3 PCA from estimatePose
% cx, cy:   robot-frame position of cube (to check approach direction)

    

    % --- LONG-SIDE CHECK (horizontal orientation) ---
    % PCA axis 1 = longest axis of the cube in XY plane
    % If the longest axis points toward the robot base (radially),
    % the gripper would have to close along the long side — impossible.
    
    long_axis_3d = double(rotation(:, 1));   % 1st PCA = longest axis
    long_axis_xy = long_axis_3d(1:2);
    long_axis_xy = long_axis_xy / (norm(long_axis_xy) + 1e-9);

    % Radial direction from robot base toward cube
    radial = [cx; cy];
    radial = radial / (norm(radial) + 1e-9);

    % If long axis is aligned with approach direction, gripper hits the long face
    alignment = abs(dot(long_axis_xy, radial));
    % alignment ~1 = long side facing robot (bad)
    % alignment ~0 = short side facing robot (good — gripper fits)

    ALIGNMENT_LIMIT = 0.85;   % cos(~32 deg) — tune to your cube aspect ratio
    if alignment > ALIGNMENT_LIMIT
        fprintf('    Long side facing robot (alignment=%.2f). UNGRASPABLE.\n', alignment);
        phi = NaN;
        graspable = false;
    end

    phi = double(phi);
end