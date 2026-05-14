%% =========================================================
% Task 8.2 – Autonomous Pick and Place with AUTO-CALIBRATION
%
% CAMERA-TO-ROBOT CALIBRATION OVERVIEW
% ─────────────────────────────────────
% The robot moves its gripper to two known positions in the
% robot frame (P1, P2).  After each move the camera takes a
% snapshot and the bright-coloured gripper marker (or the
% depth centroid) is located in the image, giving us two
% camera-frame observations (C1, C2).
%
% From just two point-pairs we can solve for:
%   sx  – sign of the camera X axis relative to robot X  (±1)
%   sy  – sign of the camera Y axis relative to robot Y  (±1)
%   dx  – X offset (robot_x = sx * cam_x + dx)
%   dy  – Y offset (robot_y = sy * cam_y + dy)
%
% The calibration takes ~15 seconds and runs once at startup.
% Results are stored in the struct CAL and used in every
% subsequent PLAN state.
%
% HOW THE SIGN IS DETERMINED
%   If moving the robot +ΔX causes the camera centroid to
%   move in the +X direction → sx = +1, else sx = −1.
%   Same logic for Y.
%
% HOW THE OFFSET IS DETERMINED
%   dx = robot_x1 − sx * cam_x1   (using the first calibration point)
%   dy = robot_y1 − sy * cam_y1
%
% REQUIREMENTS
%   • A brightly coloured sticker / rubber band on the gripper
%     tip that the HSV segmenter can isolate, OR you can use
%     the depth centroid of the gripper blob if no marker is
%     available (set USE_DEPTH_CENTROID = true below).
%   • The two calibration positions must be REACHABLE and
%     VISIBLE to the camera simultaneously.
%   • Z is NOT calibrated here – cube Z is always overridden
%     with the known physical half-height of the cube.
%
% =========================================================

%% =====================  USER CONFIG  =====================

BASKET_X   =  120;   % mm – basket centre in robot frame
BASKET_Y   =  120;   % mm

Z_HOVER    =  120;   % mm – safe transit height
Z_GRASP    =   50;   % mm – descent height for grasping
Z_PLACE    =   60;   % mm – descent height for releasing

JAW_OPEN   =   34;   % mm
JAW_CLOSED =   23;   % mm

PHI        = -pi/2;

MOVE_SPEED =   50;
PAUSE_MOVE =  2.5;
PAUSE_GRIP =  2.0;

% ── CALIBRATION POSITIONS (robot frame, mm) ──────────────
% Rules:
%   1. Both must be reachable by the arm (inside workspace).
%   2. They must differ by ≥ 60 mm in BOTH X and Y so the camera
%      motion is large enough to determine axis signs reliably.
%   3. Keep Z at Z_GRASP so the gripper is near table level
%      where the camera has a clear view.
%   4. Avoid placing them at the same θ1 angle (same X/Y ratio)
%      because the arm won't visibly move in the camera image.
%
% These defaults work for the standard Phantom X Pincher setup:
%   P1 is forward-right, P2 is forward-left (large Δ in both axes).
CAL_P1 = [-150,  50, Z_GRASP];   % [x  y  z] mm  – forward, right side
CAL_P2 = [ -50, 150, Z_GRASP];   % [x  y  z] mm  – forward, left side
% Δx = -100 mm, Δy = +100 mm → unambiguous sign determination

% ── GRIPPER MARKER COLOUR for calibration snapshot ────────
% Set to the HSV range of the sticker / tape on the gripper.
% Default: bright orange/yellow tape.
MARKER_H_LO  = 0.01;   % Lowered from 0.04
MARKER_H_HI  = 0.20;   % Raised from 0.15
MARKER_S_MIN = 0.20;   % Lowered to ignore heavy white glare
MARKER_V_MIN = 0.20;   % Lowered to ignore shadows

% Set true to use depth centroid of the closest blob instead of colour
USE_DEPTH_CENTROID =false;

% Physical half-height of one cube (mm) – used to fix Z after detection
CUBE_HALF_HEIGHT = 25.0;

% =====================  END CONFIG  =======================

global arb;

fprintf('\n=====================================================\n');
fprintf('  Task 8.2 – Auto-Calibrated Pick and Place\n');
fprintf('=====================================================\n\n');

%% =========================================================
%  STEP 0 – AUTO-CALIBRATION
% =========================================================
fprintf('=== AUTO-CALIBRATION  (takes ~15 s) ===\n');

CAL = runCalibration(arb, CAL_P1, CAL_P2, PHI, MOVE_SPEED, ...
    MARKER_H_LO, MARKER_H_HI, MARKER_S_MIN, MARKER_V_MIN, ...
    USE_DEPTH_CENTROID, PAUSE_MOVE, PAUSE_GRIP, JAW_OPEN);

fprintf('\nCalibration result:\n');
fprintf('  sx=%.0f  sy=%.0f  dx=%.1f mm  dy=%.1f mm\n', ...
    CAL.sx, CAL.sy, CAL.dx, CAL.dy);
fprintf('  Reprojection error: P1=%.1f mm  P2=%.1f mm\n', ...
    CAL.err1, CAL.err2);
fprintf('=========================================\n\n');

%% =========================================================
%  FSM  –  pick-and-place
% =========================================================
state        = 'INIT';
cube_pos     = [];
q_pick_hover  = [];
q_pick_grasp  = [];
q_place_hover = [];
q_place_grasp = [];

while true
    fprintf('[FSM] State: %s\n', state);

    switch state

        % --------------------------------------------------
        case 'INIT'
        % --------------------------------------------------
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            if ~setPosition([0,0,0,0])
                fprintf('  WARNING: Home failed.\n');
            end
            pause(PAUSE_MOVE);
            state = 'PERCEIVE';

        % --------------------------------------------------
        case 'PERCEIVE'
        % --------------------------------------------------
            fprintf('  Running perception pipeline...\n');

            [ptCloud, color_frame, intrinsics] = acquirePointCloud();
            mask = segmentObject(color_frame);
            objCloud = maskPointCloud(ptCloud, mask, intrinsics);

            if objCloud.Count == 0
                fprintf('  No red object detected.\n');
                state = 'ERROR'; continue
            end

            [clusters, ~, ~] = clusterObjects(objCloud);
            if isempty(clusters)
                fprintf('  Clustering failed.\n');
                state = 'ERROR'; continue
            end

            poses = estimatePose(clusters);
            if isempty(poses) || any(isnan(poses{1}.position))
                fprintf('  Pose estimation failed.\n');
                state = 'ERROR'; continue
            end

            % Camera frame (metres) → robot frame (mm) via calibration
            cam_x_m = poses{1}.position(1);
            cam_y_m = poses{1}.position(2);
            
            % --- THE AXIS SWAP FIX ---
            % Because the camera is sideways, Camera Y drives Robot X!
            cube_pos(1) = CAL.sx * (cam_y_m * 1000) + CAL.dx;
            cube_pos(2) = CAL.sy * (cam_x_m * 1000) + CAL.dy;
            cube_pos(3) = CUBE_HALF_HEIGHT;   % fixed physical Z

            fprintf('  Cube at robot XY: [%.1f, %.1f] mm  Z=%.1f mm\n', ...
                cube_pos(1), cube_pos(2), cube_pos(3));

            % Visual confirmation
            figure(1);
            subplot(1,2,1); imshow(color_frame); title('Camera view');
            subplot(1,2,2); imshow(mask);        title('Red mask');
            drawnow;

            state = 'PLAN';

        % --------------------------------------------------
        case 'PLAN'
        % --------------------------------------------------
            fprintf('  Computing IK...\n');

            cx = cube_pos(1);
            cy = cube_pos(2);

            % ── Sanity-check the coordinate before calling IK ──
            % Phantom X Pincher max horizontal reach ≈ 286 mm (a2+a3+a4)
            % Add 10 % margin for IK filter headroom.
            MAX_REACH = 260;   % mm – conservative safe limit
            horiz = sqrt(cx^2 + cy^2);
            if horiz > MAX_REACH
                fprintf(['  COORDINATE OUT OF REACH: [%.1f, %.1f] mm is %.1f mm from base\n' ...
                         '  (limit %.0f mm).  Calibration likely failed — rerun.\n'], ...
                         cx, cy, horiz, MAX_REACH);
                state = 'ERROR'; continue
            end

            current_servo = [arb.getpos(1), arb.getpos(2), ...
                             arb.getpos(3), arb.getpos(4)];
            current_dh    = servo2dh(current_servo);

            q_pick_hover = my_findSolution(cx, cy, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  No IK for pick-hover.\n');
                state = 'ERROR'; continue
            end

            q_pick_grasp = my_findSolution(cx, cy, Z_GRASP, PHI, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  No IK for pick-grasp.\n');
                state = 'ERROR'; continue
            end

            q_place_hover = my_findSolution(BASKET_X, BASKET_Y, Z_HOVER, PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  No IK for place-hover.\n');
                state = 'ERROR'; continue
            end

            q_place_grasp = my_findSolution(BASKET_X, BASKET_Y, Z_PLACE, PHI, q_place_hover);
            if isempty(q_place_grasp)
                fprintf('  No IK for place-grasp.\n');
                state = 'ERROR'; continue
            end

            fprintf('  All waypoints OK.\n');
            state = 'APPROACH_PICK';

        % --------------------------------------------------
        case 'APPROACH_PICK'
        % --------------------------------------------------
            % Step 1 – spin base, arm stays vertical
            setPosition([q_pick_hover(1), 0, 0, 0]);
            pause(1.5);
            % Step 2 – lower to hover
            if ~setPosition(q_pick_hover)
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'GRASP';

        % --------------------------------------------------
        case 'GRASP'
        % --------------------------------------------------
            if ~setPosition(q_pick_grasp)
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            positionJaw(JAW_CLOSED);
            pause(PAUSE_GRIP);
            state = 'LIFT';

        % --------------------------------------------------
        case 'LIFT'
        % --------------------------------------------------
            if ~setPosition(q_pick_hover)
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'APPROACH_PLACE';

        % --------------------------------------------------
        case 'APPROACH_PLACE'
        % --------------------------------------------------
            % Step 1 – spin base toward basket, keep arm lifted
            q_spin = [q_place_hover(1), q_pick_hover(2), ...
                      q_pick_hover(3),  q_pick_hover(4)];
            setPosition(q_spin);
            pause(1.5);
            % Step 2 – extend to basket hover
            if ~setPosition(q_place_hover)
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'RELEASE';

        % --------------------------------------------------
        case 'RELEASE'
        % --------------------------------------------------
            if ~setPosition(q_place_grasp)
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
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
            fprintf('  FAILURE: FSM hit ERROR state.\n');
            fprintf('============================================\n\n');
            break;

        otherwise
            fprintf('  Unknown state "%s" – aborting.\n', state);
            break;
    end
end


%% =========================================================
%%  AUTO-CALIBRATION ROUTINE
%% =========================================================
function CAL = runCalibration(arb, P1, P2, phi, spd, ...
        h_lo, h_hi, s_min, v_min, use_depth, ...
        pause_move, pause_grip, jaw_open)
%
% Moves the gripper to P1 and P2, takes a camera snapshot at
% each position, finds the gripper centroid in camera space,
% and solves for the four calibration parameters:
%   sx, sy  – axis sign conventions  (±1)
%   dx, dy  – translation offsets    (mm)
%
% robot_x = sx * cam_x_mm + dx
% robot_y = sy * cam_y_mm + dy

    fprintf('  Opening jaw and homing...\n');
    positionJaw(jaw_open);
    pause(pause_grip);
    setPosition([0,0,0,0]);
    pause(pause_move);

    % ── Move to calibration position 1 ──────────────────
    fprintf('  Moving to calibration position 1: [%.0f, %.0f, %.0f] mm\n', ...
        P1(1), P1(2), P1(3));

    q1 = my_findSolution(P1(1), P1(2), P1(3), phi, [0,0,0,0]);
    if isempty(q1)
        error('Calibration FAILED: P1 [%.0f %.0f %.0f] unreachable.', ...
            P1(1), P1(2), P1(3));
    end

    % Two-step approach for P1
    setPosition([q1(1), 0, 0, 0]);
    pause(1.5);
    setPosition(q1);
    pause(pause_move);

    % Snapshot at P1
    [ptCloud1, cf1, intr1] = acquirePointCloud();
    C1 = getGripperCentroid(ptCloud1, cf1, intr1, ...
        h_lo, h_hi, s_min, v_min, use_depth);

    fprintf('  Camera centroid at P1: cam_x=%.1f mm  cam_y=%.1f mm\n', ...
        C1(1), C1(2));

    % ── Move to calibration position 2 ──────────────────
    fprintf('  Moving to calibration position 2: [%.0f, %.0f, %.0f] mm\n', ...
        P2(1), P2(2), P2(3));

    q2 = my_findSolution(P2(1), P2(2), P2(3), phi, q1);
    if isempty(q2)
        error('Calibration FAILED: P2 [%.0f %.0f %.0f] unreachable.', ...
            P2(1), P2(2), P2(3));
    end

    setPosition(q2);
    pause(pause_move);

    % Snapshot at P2
    [ptCloud2, cf2, intr2] = acquirePointCloud();
    C2 = getGripperCentroid(ptCloud2, cf2, intr2, ...
        h_lo, h_hi, s_min, v_min, use_depth);

    fprintf('  Camera centroid at P2: cam_x=%.1f mm  cam_y=%.1f mm\n', ...
        C2(1), C2(2));

    % Return home
    setPosition([0,0,0,0]);
    pause(pause_move);

    % ── Solve for sx, sy ────────────────────────────────
    % Moving robot from P1 → P2:
    %   Δrobot_x = P2(1) − P1(1)
    %   Δcam_x   = C2(1) − C1(1)   (mm in camera frame)
    % If they have the same sign  → sx = +1
    % If they have opposite signs → sx = −1
    % ── Solve for sx, sy (WITH AXIS SWAP FIX) ────────────────────
    delta_robot_x = P2(1) - P1(1);
    delta_robot_y = P2(2) - P1(2);
    delta_cam_x   = C2(1) - C1(1);
    delta_cam_y   = C2(2) - C1(2);
    
    % Robot X tracks Camera Y. Robot Y tracks Camera X.
    sx = sign(delta_robot_x) * sign(delta_cam_y);   
    sy = sign(delta_robot_y) * sign(delta_cam_x);   
    
    % ── Solve for dx, dy using swapped points ──
    dx = 0.5 * ((P1(1) - sx*C1(2)) + (P2(1) - sx*C2(2)));
    dy = 0.5 * ((P1(2) - sy*C1(1)) + (P2(2) - sy*C2(1)));
    
    % ── Reprojection error check ─────────────────────────
    pred_x1 = sx * C1(2) + dx;   pred_y1 = sy * C1(1) + dy;
    pred_x2 = sx * C2(2) + dx;   pred_y2 = sy * C2(1) + dy;
    err1 = sqrt((pred_x1 - P1(1))^2 + (pred_y1 - P1(2))^2);
    err2 = sqrt((pred_x2 - P2(1))^2 + (pred_y2 - P2(2))^2);
    
    if err1 > 20 || err2 > 20
        warning(['Calibration reprojection error is large (%.1f / %.1f mm).\n' ...
                 'Check that the gripper marker is visible at both positions.'], ...
                 err1, err2);
    end

    CAL.sx   = sx;
    CAL.sy   = sy;
    CAL.dx   = dx;
    CAL.dy   = dy;
    CAL.err1 = err1;
    CAL.err2 = err2;
end


function cam_xy_mm = getGripperCentroid(ptCloud, color_frame, intrinsics, ...
        h_lo, h_hi, s_min, v_min, use_depth)
% Find the 2D centroid of the gripper marker in CAMERA space (mm).
% Returns [cam_x_mm, cam_y_mm] where the origin is the camera optical
% axis and units are millimetres (derived from the point cloud depth).
%
% Two modes:
%   use_depth=false → colour segmentation of marker sticker
%   use_depth=true  → closest point cluster in the depth image

    if use_depth
        % ── DEPTH-DIFFERENCE GRIPPER FINDER (GOD MODE) ────────
        xyz = ptCloud.Location;
        ok  = ~any(isnan(xyz),2) & xyz(:,3) > 0.05 & xyz(:,3) < 0.60;
        xyz = xyz(ok,:);
        if isempty(xyz)
            error('getGripperCentroid: no valid depth points in [0.05, 0.60] m range.');
        end
        
        % Grab a much thicker slice of the arm (shallowest 30%)
        z_cut    = prctile(xyz(:,3), 30);
        fore     = xyz(xyz(:,3) <= z_cut, :);
        
        % BYPASS: No XY radius filter! 
        % Since the table is clear, whatever is closest to the camera IS the arm.
        cx_m     = mean(fore(:,1));
        cy_m     = mean(fore(:,2));
    else
        % Colour segmentation of the marker
        rgb = im2double(color_frame);
        hsv = rgb2hsv(rgb);
        h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);
        marker_mask = (h >= h_lo) & (h <= h_hi) & (s > s_min) & (v > v_min);
        marker_mask = imopen(marker_mask,  strel('disk',3));
        marker_mask = imclose(marker_mask, strel('disk',5));
        marker_mask = bwareaopen(marker_mask, 100);

        if ~any(marker_mask(:))
            error(['getGripperCentroid: marker not found in image.\n' ...
                   'Check MARKER_H_LO/HI, or set USE_DEPTH_CENTROID=true.']);
        end

        % Mask the point cloud with the colour mask
        markerCloud = maskPointCloud(ptCloud, marker_mask, intrinsics);

        xyz  = markerCloud.Location;
        xyz  = xyz(~any(isnan(xyz),2) & xyz(:,3)>0, :);
        if isempty(xyz)
            error('getGripperCentroid: masked cloud empty.');
        end
        cx_m = mean(xyz(:,1));
        cy_m = mean(xyz(:,2));
    end

    % Convert metres → mm
    cam_xy_mm = [cx_m * 1000, cy_m * 1000];
end


%% =========================================================
%%  MOTION HELPERS
%%  NOTE: setPosition and positionJaw are NOT defined here.
%%  They are called from your existing setPosition.m and
%%  positionJaw.m files on the MATLAB path.
%%  setPosition(jointAngles)  – 1×4 DH angles, no speed arg
%%  positionJaw(position_mm)  – jaw gap in mm
%% =========================================================


%% =========================================================
%%  PERCEPTION PIPELINE
%% =========================================================

function [ptCloud, color_frame, intrinsics] = acquirePointCloud()
    pipe    = realsense.pipeline();
    profile = pipe.start();
    cleanup = onCleanup(@() safestop(pipe)); %#ok<NASGU>

    dev          = profile.get_device();
    ds           = dev.first('depth_sensor').get_depth_scale();
    dstream      = profile.get_stream(realsense.stream.depth) ...
                       .as('video_stream_profile');
    di           = dstream.get_intrinsics();

    for i = 1:5, fs = pipe.wait_for_frames(); end

    align = realsense.align(realsense.stream.depth);
    fs    = align.process(fs);

    df   = fs.get_depth_frame();
    dd   = double(df.get_data());
    depth_frame = permute(reshape(dd,[df.get_width(),df.get_height()]),[2 1]);
    dm  = depth_frame .* ds;
    depth_frame(dm < 0.20 | dm > 0.90) = 0;

    cf  = fs.get_color_frame();
    cd  = cf.get_data();
    color_frame = permute(reshape(cd,[3,cf.get_width(),cf.get_height()]),[3 2 1]);

    intrinsics = cameraIntrinsics([di.fx,di.fy],[di.ppx,di.ppy],size(depth_frame));

    raw = pcfromdepth(depth_frame,1/ds,intrinsics,'ColorImage',color_frame);
    ds2 = pcdownsample(raw,'gridAverage',0.002);
    [ptCloud,~] = pcdenoise(ds2,'NumNeighbors',20,'Threshold',2.0);
end

function safestop(p), try, p.stop(); catch, end; end

function mask = segmentObject(color_frame)
    rgb = im2double(color_frame);
    hsv = rgb2hsv(rgb);
    h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);
    mask = (h < 0.08 | h > 0.92) & (s > 0.25) & (v > 0.20);
    mask = imopen(mask,  strel('disk',3));
    mask = imclose(mask, strel('disk',5));
    mask = imfill(mask,'holes');
    mask = bwareaopen(mask, 500);
end

function objCloud = maskPointCloud(ptCloud, mask, intrinsics)
    xyz = ptCloud.Location; rgb = ptCloud.Color;
    ok  = ~any(isnan(xyz),2) & (xyz(:,3)>0);
    xyz = xyz(ok,:); rgb = rgb(ok,:);
    if isempty(xyz), objCloud = pointCloud(zeros(1,3)); return; end

    fx = intrinsics.FocalLength(1);  fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1); cy = intrinsics.PrincipalPoint(2);
    col = round(xyz(:,1)./xyz(:,3).*fx + cx);
    row = round(xyz(:,2)./xyz(:,3).*fy + cy);
    [H,W] = size(mask);
    ib  = col>=1 & col<=W & row>=1 & row<=H;
    col=col(ib); row=row(ib); xyz=xyz(ib,:); rgb=rgb(ib,:);
    idx = sub2ind([H,W],row,col);
    in  = mask(idx);
    if isempty(xyz(in,:)), objCloud = pointCloud(zeros(1,3)); return; end
    objCloud = pointCloud(xyz(in,:),'Color',rgb(in,:));
end

function [clusters,labels,K] = clusterObjects(ptCloud, K_override)
    MIN_PTS=30; K_MAX=8; REPS=5; ITER=200;
    xyz = ptCloud.Location;
    xyz = xyz(~any(isnan(xyz),2),:);
    N   = size(xyz,1);
    if N < MIN_PTS, clusters={}; labels=[]; K=0; return; end
    if nargin>=2 && ~isempty(K_override)
        K = max(1,round(K_override));
    else
        Km = min(K_MAX,floor(N/MIN_PTS));
        if Km < 2, K=1;
        else
            wcss=zeros(1,Km);
            for k=1:Km
                [~,~,sd]=kmeans(xyz,k,'Distance','sqeuclidean',...
                    'Replicates',REPS,'MaxIter',ITER,'Display','off');
                wcss(k)=sum(sd);
            end
            w=wcss/wcss(1); ks=1:Km;
            p1=[ks(1),w(1)]; p2=[ks(end),w(end)]; d=zeros(1,Km);
            for i=1:Km
                p=[ks(i),w(i)];
                d(i)=abs(cross2d(p2-p1,p1-p))/norm(p2-p1);
            end
            [~,K]=max(d);
        end
    end
    [labels,~]=kmeans(xyz,K,'Distance','sqeuclidean',...
        'Replicates',REPS,'MaxIter',ITER,'Display','off');
    vm=~any(isnan(ptCloud.Location),2);
    rgb=ptCloud.Color; rgb=rgb(vm,:);
    clusters={};
    for k=1:K
        idx=find(labels==k);
        if numel(idx)<MIN_PTS, continue; end
        clusters{end+1}=pointCloud(xyz(idx,:),'Color',rgb(idx,:)); %#ok<AGROW>
    end
end

function d=cross2d(a,b), d=a(1)*b(2)-a(2)*b(1); end

function poses = estimatePose(clusters)
    poses = cell(1,length(clusters));
    for k=1:length(clusters)
        xyz=clusters{k}.Location;
        xyz=xyz(~any(isnan(xyz),2),:);
        if size(xyz,1)<3
            poses{k}.position=[NaN NaN NaN];
            poses{k}.rotation=eye(3);
            continue
        end
        poses{k}.position=mean(xyz,1);
        [R,~,~]=pca(xyz);
        poses{k}.rotation=R;
    end
end


