%% =========================================================
% Task 8.2 – Autonomous Pick and Place with FSM
%
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
BASKET_X   =   0;      % mm  – change to your basket's X
BASKET_Y   = 200;      % mm  – change to your basket's Y

% ---- Heights (mm) -------------------------------------------
Z_HOVER    = 150;      % safe transit height above table
Z_GRASP    =  30;      % height at which jaw closes on cube
                       % (tune: ~half cube height above table)
Z_PLACE    =  60;      % height at which jaw opens over basket

% ---- Gripper widths (mm jaw gap) ----------------------------
JAW_OPEN   =  33;      % fully open  (max ~34.59 mm)
JAW_CLOSED =  24;      % closed on cube – tune to cube width

% ---- End-effector pitch during manipulation -----------------
PHI        = -pi/2;    % gripper pointing straight down

% ---- Motion speed -------------------------------------------
MOVE_SPEED = 50;       % setpos speed argument

% ---- Pauses (seconds) ---------------------------------------
PAUSE_MOVE  = 2.5;     % wait after an arm motion
PAUSE_GRIP  = 1.0;     % wait after a gripper command

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
            if ~setPosition(home_dh)
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
            cube_pos = poses{1}.position * 1000;  % [x y z] mm

            fprintf('  Cube detected at: X=%.1f  Y=%.1f  Z=%.1f mm\n', ...
                cube_pos(1), cube_pos(2), cube_pos(3));

            state = 'PLAN';

        % --------------------------------------------------
        case 'PLAN'
        % --------------------------------------------------
            fprintf('  Computing IK for all waypoints...\n');

            cx = cube_pos(1);
            cy = cube_pos(2);

            % Current joint config for IK seed
            current_servo = [arb.getpos(1), arb.getpos(2), ...
                             arb.getpos(3), arb.getpos(4)];
            current_dh    = servo2dh(current_servo);

            % ---- Pick waypoints ----------------------------
            q_pick_hover = findSolution(cx, cy, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  ERROR: No IK for pick-hover.\n');
                state = 'ERROR'; continue
            end

            q_pick_grasp = findSolution(cx, cy, Z_GRASP, PHI, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  ERROR: No IK for pick-grasp.\n');
                state = 'ERROR'; continue
            end

            % ---- Place waypoints ---------------------------
            q_place_hover = findSolution(BASKET_X, BASKET_Y, Z_HOVER, ...
                                         PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  ERROR: No IK for place-hover.\n');
                state = 'ERROR'; continue
            end

            q_place_grasp = findSolution(BASKET_X, BASKET_Y, Z_PLACE, ...
                                         PHI, q_place_hover);
            if isempty(q_place_grasp)
                fprintf('  ERROR: No IK for place-grasp.\n');
                state = 'ERROR'; continue
            end

            fprintf('  All waypoints planned successfully.\n');
            state = 'APPROACH_PICK';

        % --------------------------------------------------
        case 'APPROACH_PICK'
        % --------------------------------------------------
            fprintf('  Moving to hover above cube...\n');

            if ~setPosition(q_pick_hover)
                fprintf('  ERROR: Motion failed at APPROACH_PICK.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            state = 'GRASP';

        % --------------------------------------------------
        case 'GRASP'
        % --------------------------------------------------
            fprintf('  Descending to cube...\n');

            if ~setPosition(q_pick_grasp)
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

            if ~setPosition(q_pick_hover)
                fprintf('  ERROR: Motion failed during lift.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            state = 'APPROACH_PLACE';

        % --------------------------------------------------
        case 'APPROACH_PLACE'
        % --------------------------------------------------
            fprintf('  Moving to hover above basket...\n');

            if ~setPosition(q_place_hover)
                fprintf('  ERROR: Motion failed at APPROACH_PLACE.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            state = 'RELEASE';

        % --------------------------------------------------
        case 'RELEASE'
        % --------------------------------------------------
            fprintf('  Lowering cube into basket...\n');

            if ~setPosition(q_place_grasp)
                fprintf('  ERROR: Motion failed lowering into basket.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            fprintf('  Opening jaw – releasing cube...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);

            % Retract back to hover so we clear the basket rim
            setPosition(q_place_hover);
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
%  LOCAL HELPER – setPosition (from motion pipeline)
%  Converts DH angles → servo ticks and sends to robot.
%  Returns 1 on success, 0 on failure.
% =========================================================
function success = setPosition(jointAngles)
    global arb;

    % Safety joint limits (radians)
    limit_min = [-pi,  -1.9, -1.8, -1.8];
    limit_max = [ pi,   1.9,  1.8,  1.8];

    for i = 1:4
        if jointAngles(i) < limit_min(i) || jointAngles(i) > limit_max(i)
            fprintf('  Error: Joint %d (%.2f rad) exceeds safety limits.\n', ...
                i, jointAngles(i));
            success = 0;
            return;
        end
    end

    try
        servo_ticks    = dh2servo(jointAngles);
        current_gripper = arb.getpos(5);
        final_command  = [servo_ticks(1), servo_ticks(2), ...
                          servo_ticks(3), servo_ticks(4), current_gripper];
        arb.setpos(final_command, [MOVE_SPEED, MOVE_SPEED, ...
                                   MOVE_SPEED, MOVE_SPEED, MOVE_SPEED]);
        success = 1;
    catch err
        fprintf('  Communication error: %s\n', err.message);
        success = 0;
    end
end


%% =========================================================
%  LOCAL HELPER – positionJaw  (from motion pipeline)
%  Controls the parallel gripper jaw opening in mm.
% =========================================================
function success = positionJaw(position)
    global arb;

    L1 = 8.68;
    L2 = 25.91;
    min_pos = L2 - L1;   % ~17.23 mm
    max_pos = L1 + L2;   % ~34.59 mm

    if position < min_pos || position > max_pos
        fprintf('  ERROR: Jaw position %.2f mm out of range [%.2f, %.2f].\n', ...
            position, min_pos, max_pos);
        success = 0;
        return;
    end

    cos_theta1 = (L1^2 + position^2 - L2^2) / (2 * L1 * position);
    cos_theta1 = max(min(cos_theta1, 1), -1);
    theta1     = acos(cos_theta1);

    experimental_offset = 0.0;
    direction           = 1;
    gripper_servo_angle = (direction * theta1) + experimental_offset;

    try
        current_arm   = [arb.getpos(1), arb.getpos(2), ...
                         arb.getpos(3), arb.getpos(4)];
        full_command  = [current_arm, gripper_servo_angle];
        arb.setpos(full_command, 50);
        success = 1;
    catch
        disp('  ERROR: Failed to communicate with gripper.');
        success = 0;
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

    pipe    = realsense.pipeline();
    profile = pipe.start();

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

    pipe.stop();

    % Extract depth frame
    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data, ...
        [depth.get_width(), depth.get_height()]), [2 1]);

    % Clamp depth to workspace range (metres)
    MIN_DEPTH_M = 0.20;
    MAX_DEPTH_M = 0.90;
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
    mask = (h < 0.05 | h > 0.95) & (s > 0.4) & (v > 0.3);

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


%% =========================================================
%  MOTION PIPELINE FUNCTIONS
%  (copied / condensed from fullpipeline.pdf)
% =========================================================

function solutions = findJointAngles(x, y, z, phi)
% Analytical IK for the Phantom X Pincher (4-DOF).
% Returns an N×4 matrix where each row is [th1 th2 th3 th4].

    d1 = 145; a2 = 105; a3 = 106; a4 = 75;   % mm

    solutions = [];

    th1_forward  = atan2( y,  x);
    th1_backward = atan2(-y, -x);
    r_forward    =  sqrt(x^2 + y^2);
    r_backward   = -sqrt(x^2 + y^2);

    s            = z - d1;
    base_angles  = [th1_forward,  th1_backward];
    r_values     = [r_forward,    r_backward  ];

    for i = 1:2
        th1 = base_angles(i);
        r   = r_values(i);

        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);

        D_cos = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        if abs(D_cos) > 1
            continue
        end

        for sign = [1, -1]
            th3 = atan2(sign * sqrt(1 - D_cos^2), D_cos);
            th2 = atan2(s_bar, r_bar) - atan2(a3*sin(th3), a2+a3*cos(th3));
            th4 = phi - th2 - th3;

            th1 = mod(th1 + pi, 2*pi) - pi;
            th2 = mod(th2 + pi, 2*pi) - pi;
            th3 = mod(th3 + pi, 2*pi) - pi;
            th4 = mod(th4 + pi, 2*pi) - pi;

            solutions = [solutions; th1, th2, th3, th4]; %#ok<AGROW>
        end
    end
end

% ---------------------------------------------------------
function optimal_solution = findSolution(x, y, z, phi, currentConfig)
% Choose the best collision-free IK solution closest to
% currentConfig, applying joint-limit and collision filters.

    all_solutions = findJointAngles(x, y, z, phi);

    if isempty(all_solutions)
        disp('ERROR: Target is mathematically out of reach.');
        optimal_solution = [];
        return
    end

    limit_rad      = 150 * (pi / 180);
    valid_solutions = [];

    for i = 1:size(all_solutions, 1)
        candidate = all_solutions(i, :);

        % Filter 1: joint limits
        if any(candidate < -limit_rad) || any(candidate > limit_rad)
            continue
        end

        % Filter 2: self-collision (lightweight check)
        if checkSelfCollision(currentConfig, candidate)
            continue
        end

        valid_solutions = [valid_solutions; candidate]; %#ok<AGROW>
    end

    if isempty(valid_solutions)
        disp('ERROR: No safe solutions exist.');
        optimal_solution = [];
        return
    end

    % Pick solution with minimum weighted joint displacement
    b1 = 4; b2 = 3; b3 = 2; b4 = 1;
    weights   = [b1, b2, b3, b4];
    best_cost = inf;
    best_idx  = 1;

    for i = 1:size(valid_solutions, 1)
        candidate    = valid_solutions(i, :);
        angular_diff = mod((candidate - currentConfig) + pi, 2*pi) - pi;
        cost         = sum(weights .* abs(angular_diff));
        if cost < best_cost
            best_cost = cost;
            best_idx  = i;
        end
    end

    optimal_solution = valid_solutions(best_idx, :);
end

% ---------------------------------------------------------
function isCollision = checkSelfCollision(initial_angles, final_angles)
% Lightweight self-collision check: samples 50 intermediate
% configs using the digital twin with collision geometry.

    robot       = buildPincher();
    robot       = addPincherCollisions(robot);
    num_samples = 50;
    isCollision = false;

    for i = 1:num_samples
        fraction      = i / num_samples;
        current_config = initial_angles + fraction * (final_angles - initial_angles);
        collision_status = checkCollision(robot, current_config, ...
            'SkippedSelfCollisions', 'adjacent');
        if any(collision_status)
            isCollision = true;
            return
        end
    end
end

% ---------------------------------------------------------
function robot = buildPincher()
% Create a rigidBodyTree digital twin of the Phantom X Pincher.

    robot  = rigidBodyTree('DataFormat', 'row');
    a      = [0,   105, 106, 75];
    alpha  = [pi/2,  0,   0,  0];
    d      = [145,   0,   0,  0];

    parentName = 'base';
    for i = 1:4
        bodyName  = sprintf('link%d', i);
        jointName = sprintf('joint%d', i);
        body      = rigidBody(bodyName);
        joint     = rigidBodyJoint(jointName, 'revolute');
        setFixedTransform(joint, [a(i), alpha(i), d(i), 0], 'dh');
        body.Joint = joint;
        addBody(robot, body, parentName);
        parentName = bodyName;
    end
end

% ---------------------------------------------------------
function robot = addPincherCollisions(robot)
% Add cylinder collision geometries to the digital twin.

    d1 = 145; a2 = 105; a3 = 106; a4 = 75;
    radius = 18;

    coll1 = collisionCylinder(radius, d1);
    T1 = trvec2tform([0, -d1/2, 0]) * axang2tform([1, 0, 0, pi/2]);
    addCollision(robot.Bodies{1}, coll1, T1);

    coll2 = collisionCylinder(radius, a2);
    T2 = trvec2tform([-a2/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{2}, coll2, T2);

    coll3 = collisionCylinder(radius, a3);
    T3 = trvec2tform([-a3/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{3}, coll3, T3);

    coll4 = collisionCylinder(radius, a4);
    T4 = trvec2tform([-a4/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{4}, coll4, T4);
end

% ---------------------------------------------------------
function dhJointAngles = servo2dh(servoAngles)
% Convert raw servo angles to DH convention angles.

    offset1 = 0;
    offset2 = pi/2;
    offset3 = 0;
    offset4 = 0;

    dir1 =  1;
    dir2 = -1;
    dir3 = -1;
    dir4 = -1;

    dhJointAngles = zeros(1, 4);
    dhJointAngles(1) = (dir1 * servoAngles(1)) + offset1;
    dhJointAngles(2) = (dir2 * servoAngles(2)) + offset2;
    dhJointAngles(3) = (dir3 * servoAngles(3)) + offset3;
    dhJointAngles(4) = (dir4 * servoAngles(4)) + offset4;

    dhJointAngles = mod(dhJointAngles + pi, 2*pi) - pi;
end

% ---------------------------------------------------------
function servoAngles = dh2servo(dhJointAngles)
% Convert DH angles back to raw servo motor angles
% (exact mathematical inverse of servo2dh).

    offset1 = 0;
    offset2 = pi/2;
    offset3 = 0;
    offset4 = 0;

    dir1 =  1;
    dir2 = -1;
    dir3 = -1;
    dir4 = -1;

    servoAngles = zeros(1, 4);
    servoAngles(1) = dir1 * (dhJointAngles(1) - offset1);
    servoAngles(2) = dir2 * (dhJointAngles(2) - offset2);
    servoAngles(3) = dir3 * (dhJointAngles(3) - offset3);
    servoAngles(4) = dir4 * (dhJointAngles(4) - offset4);

    servoAngles = mod(servoAngles + pi, 2*pi) - pi;
end
