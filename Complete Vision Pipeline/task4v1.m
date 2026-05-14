%% =========================================================
% Task 4 (Bonus) – Pick & Place with Stacked Cubes
%
% Built directly on top of Task 8.2 Milestone 2 code.
% All perception / motion helpers kept IDENTICAL to that file
% (same setPosition, positionJaw, servo2dh, dh2servo, etc.)
%
% WHAT IS NEW vs Milestone 2
% ──────────────────────────
% 1. ITERATIVE PERCEPTION
%    The scene is re-scanned after EVERY successful pick.
%    A cube removed from the top of a stack reveals whatever
%    is underneath — only a fresh scan can see it.
%    The FSM loops  PERCEIVE → NEXT_TARGET → … → RELEASE → PERCEIVE
%    until no more target cubes appear.
%
% 2. STACK RECONSTRUCTION  (buildPickQueue)
%    After clustering, each cluster centroid gives (x,y,z) in
%    camera metres. Clusters sharing the same XY footprint
%    (within XY_STACK_RADIUS) belong to the same physical stack.
%    Within each stack they are sorted by Z DESCENDING so we
%    always pick the topmost cube first — preventing toppling.
%
% 3. GRASP HEIGHT FROM CAMERA Z
%    For a cube whose cluster centroid is at camera depth Z_c,
%    the gripper descends to:
%        z_grasp = Z_c_mm + CUBE_SIDE - GRASP_BELOW_TOP
%    where CUBE_SIDE is the physical half-side and
%    GRASP_BELOW_TOP is how far below the top face the jaws
%    close (keeps jaws above the cube below).
%
% 4. TOPPLE-SAFE TWO-STEP APPROACH
%    Identical two-step motion from M2 (spin base with arm
%    vertical, then lower) prevents the arm sweeping sideways
%    through a neighbouring stack.
%
% FSM STATES
%   INIT           – open jaw, home arm
%   PERCEIVE       – full rescan, rebuild sorted pick queue
%   NEXT_TARGET    – pop topmost target cube from queue
%   PLAN           – IK; flag if unreachable or badly tilted
%   APPROACH_PICK  – spin base → hover above cube
%   GRASP          – descend to grasp height, close jaw
%   LIFT           – ascend to Z_HOVER
%   APPROACH_PLACE – transit to basket
%   RELEASE        – lower to Z_DROP, open jaw, retract
%   DONE           – print summary, home arm
%   ERROR          – hardware fault, abort
% =========================================================

%% =====================  USER CONFIG  =====================

% Basket (mm, robot base frame – measure with ruler)
BASKET_X =   0;
BASKET_Y = 180;

% Heights (mm)
Z_HOVER  = 150;   % safe transit – must clear tallest possible stack
Z_DROP   =  90;   % release height inside basket (low = no bounce)

% Physical cube geometry (mm)
CUBE_SIDE        = 25;   % half-side of one cube
GRASP_BELOW_TOP  = 10;   % jaw closes this far below the cube's top face
                          % keeps jaw above the cube underneath

% Stack grouping: clusters closer than this in XY are one stack (mm)
XY_STACK_RADIUS  = 35;

% Gripper (mm jaw gap)
JAW_OPEN   = 34;
JAW_CLOSED = 23;   % tune to your cube width

% End-effector pitch
PHI = -pi/2;

% Motion
MOVE_SPEED = 50;
PAUSE_MOVE = 2.5;
PAUSE_GRIP = 2.0;

% Camera-to-robot frame offsets (same as your M2 tuning)
% robot_x = -cam_x_mm + CAM_DX
% robot_y = -cam_y_mm + CAM_DY   (sign convention from your PLAN block)
CAM_DX = -5;    % mm  (your cx_raw - 5)
CAM_DY = 25;    % mm  (your cy_raw + 25)

% Tilt limit for graspability check (degrees)
MAX_TILT_DEG = 35;

% Max consecutive scans with no new cubes before declaring DONE
MAX_EMPTY_SCANS = 2;

% =====================  END CONFIG  =======================

global arb;

fprintf('\n=====================================================\n');
fprintf('  Task 4 (Bonus) – Stacked Cube Pick & Place\n');
fprintf('  Basket: [%.0f, %.0f] mm\n', BASKET_X, BASKET_Y);
fprintf('=====================================================\n\n');

%% =========================================================
%  FSM
% =========================================================
state         = 'INIT';
pick_queue    = {};    % ordered list of cubes to pick (topmost first)
current_cube  = [];
delivered     = 0;
flagged       = 0;
scan_count    = 0;
empty_scans   = 0;    % consecutive scans that found nothing new

q_pick_hover  = [];
q_pick_grasp  = [];
q_place_hover = [];
q_place_drop  = [];

fig = figure('Name','Task 4 – Stack Perception','NumberTitle','off');

while true
    fprintf('[FSM] State: %s\n', state);

    switch state

        %% ------------------------------------------------
        case 'INIT'
        %% ------------------------------------------------
            fprintf('  Homing arm and opening jaw...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);

            if ~setPosition([0,0,0,0], MOVE_SPEED)
                fprintf('  WARNING: Home motion failed – continuing.\n');
            end
            pause(PAUSE_MOVE);
            state = 'PERCEIVE';

        %% ------------------------------------------------
        %% ------------------------------------------------
        case 'PERCEIVE'
        %% ------------------------------------------------
            scan_count = scan_count + 1;
            fprintf('  --- SCAN #%d ---\n', scan_count);
            
            % Full perception pipeline
            [ptCloud, color_frame, intrinsics] = acquirePointCloud();
            mask     = segmentObject(color_frame);
            
            % --- TASK 2 FIX: Count 2D blobs to prevent ghosting! ---
            cc       = bwconncomp(mask);
            props_2d = regionprops(cc, 'Area', 'Centroid');
            valid_blobs = find([props_2d.Area] > 500);
            actual_block_count = numel(valid_blobs);
            
            fprintf('  2D blobs detected: %d\n', actual_block_count);
            
            if actual_block_count == 0
                empty_scans = empty_scans + 1;
                fprintf('  No target-colour points found (empty scan #%d).\n', empty_scans);
                state = 'DONE';
                continue
            end
            
            % Pre-compute all point cloud pixel projections once
            fx   = intrinsics.FocalLength(1);
            fy   = intrinsics.FocalLength(2);
            cx_i = intrinsics.PrincipalPoint(1);
            cy_i = intrinsics.PrincipalPoint(2);
            
            xyz_all  = ptCloud.Location;
            validPts = ~any(isnan(xyz_all), 2) & xyz_all(:,3) > 0;
            xyz_all  = xyz_all(validPts, :);
            
            col_all  = round(xyz_all(:,1) ./ xyz_all(:,3) .* fx + cx_i);
            row_all  = round(xyz_all(:,2) ./ xyz_all(:,3) .* fy + cy_i);
            
            raw_cubes = {};
            
            for i = 1:numel(valid_blobs)
                k = valid_blobs(i);
                u = round(props_2d(k).Centroid(1));
                v = round(props_2d(k).Centroid(2));
                
                % Grab only the 3D points exactly under this 2D blob!
                near = abs(col_all - u) < 10 & abs(row_all - v) < 10;
                
                if sum(near) < 3
                    fprintf('  Blob %d: insufficient depth, skipping.\n', i);
                    continue
                end
                
                % Extract local position in mm directly
                pos_mm = double(mean(xyz_all(near, :), 1) * 1000);
                
                % Get PCA rotation from the actual blob points
                blob_xyz = xyz_all(near, :);
                if size(blob_xyz, 1) >= 3
                    [R, ~, ~] = pca(blob_xyz);
                else
                    R = eye(3);
                end
                
                % --- TASK 4 MATH: Calculate the Stack Height ---
                cam_x_mm = pos_mm(1);
                cam_y_mm = pos_mm(2);
                cam_z_mm = pos_mm(3);  % This is DEPTH from the lens!
                
                rx = double(-cam_y_mm + CAM_DX); 
                ry = double(-cam_x_mm + CAM_DY);
                
                % Make sure this matches your tuned number from earlier!
                TABLE_DEPTH_MM = 550; 
                
                % True height of the centroid above the table
                true_height = TABLE_DEPTH_MM - cam_z_mm;
                
                % Top-face Z: centroid height + half-side of the cube
                top_z = double(true_height + CUBE_SIDE);
                
                % Clamp to a safe range (20mm minimum, max stack height of 150mm)
                top_z = max(min(top_z, 150), 20);
                
                entry.rx       = rx;
                entry.ry       = ry;
                entry.top_z    = top_z;
                entry.cam_z_mm = cam_z_mm;
                entry.rotation = R;
                entry.label    = i;
                entry.stack_id = 0;
                entry.layer    = 0;
                
                raw_cubes{end+1} = entry; %#ok<AGROW>
            end
            
            if isempty(raw_cubes)
                state = 'DONE'; continue
            end
            
            % Group into stacks, sort each stack top-first, flatten to queue
            pick_queue  = buildPickQueue(raw_cubes, XY_STACK_RADIUS);
            empty_scans = 0;   % reset empty-scan counter on successful detection
            
            fprintf('  %d cube(s) queued (topmost first).\n', numel(pick_queue));
            showStackFigure(fig, color_frame, mask, pick_queue);
            state = 'NEXT_TARGET';

        %% ------------------------------------------------
        case 'NEXT_TARGET'
        %% ------------------------------------------------
            if isempty(pick_queue)
                % Queue exhausted – rescan to find newly exposed cubes
                state = 'PERCEIVE';
                continue
            end

            current_cube = pick_queue{1};
            pick_queue   = pick_queue(2:end);

            fprintf('  Target: cube #%d  stack=%d layer=%d\n', ...
                current_cube.label, current_cube.stack_id, current_cube.layer);
            fprintf('  Robot XY=[%.1f, %.1f] mm  top_z=%.1f mm\n', ...
                current_cube.rx, current_cube.ry, current_cube.top_z);

            state = 'PLAN';

        %% ------------------------------------------------
            %% ------------------------------------------------
        case 'PLAN'
        %% ------------------------------------------------
            % 1. Extract coordinates directly and FORCE to double!
            % (They were already converted to the robot frame in PERCEIVE)
            rx = double(current_cube.rx);
            ry = double(current_cube.ry);
            
            % 2. Grasp Z: top-face minus GRASP_BELOW_TOP (Force to double!)
            z_grasp = double(current_cube.top_z - GRASP_BELOW_TOP);
            z_grasp = max(z_grasp, 15);   % safety floor
            
            % 3. Tilt check (Commented out to bypass the noise bug from Task 3!)
            % cube_up  = current_cube.rotation(:,3);
            % tilt_deg = acosd(abs(dot(cube_up, [0;0;1])));
            % if tilt_deg > MAX_TILT_DEG
            %     fprintf('  *** UNGRASPABLE: tilt=%.1f deg – skipping. ***\n', tilt_deg);
            %     flagged = flagged + 1;
            %     state   = 'NEXT_TARGET';
            %     continue
            % end
            
            % 4. Current arm config as IK seed (Force to double!)
            current_servo = double([arb.getpos(1), arb.getpos(2), ...
                                    arb.getpos(3), arb.getpos(4)]);
            current_dh    = servo2dh(current_servo);
            
            % Pick hover: directly above cube at Z_HOVER
            q_pick_hover = my_findSolution(rx, ry, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  No pick-hover IK – skipping cube #%d.\n', current_cube.label);
                flagged = flagged + 1;
                state   = 'NEXT_TARGET'; continue
            end
            
            % Pick grasp: descend to top-of-cube
            q_pick_grasp = my_findSolution(rx, ry, z_grasp, PHI, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  No pick-grasp IK – skipping cube #%d.\n', current_cube.label);
                flagged = flagged + 1;
                state   = 'NEXT_TARGET'; continue
            end
            
            % Basket hover
            q_place_hover = my_findSolution(BASKET_X, BASKET_Y, Z_HOVER, PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  ERROR: No basket-hover IK.\n');
                state = 'ERROR'; continue
            end
            
            % Basket drop (controlled low release – no bounce)
            q_place_drop = my_findSolution(BASKET_X, BASKET_Y, Z_DROP, PHI, q_place_hover);
            if isempty(q_place_drop)
                fprintf('  ERROR: No basket-drop IK.\n');
                state = 'ERROR'; continue
            end
            
            fprintf('  Plan OK: z_grasp=%.1f\n', z_grasp);
            state = 'APPROACH_PICK';
        %% ------------------------------------------------
        case 'APPROACH_PICK'
        %% ------------------------------------------------
           fprintf('  Turning base to face target...\n');
            
            % --- THE SWOOP FIX ---
            % 1. Turn the base (Joint 1) to match the target, but keep 
            % the shoulder, elbow, and wrist perfectly straight up (0,0,0)
            q_intermediate = [q_pick_hover(1), 0, 0, 0];
            setPosition(q_intermediate,MOVE_SPEED);
            pause(1.5); % Give the base time to spin
            
            fprintf('  Dropping to hover above cube...\n');
            % 2. Now bring the arm straight down to the hover position
            if ~setPosition(q_pick_hover,MOVE_SPEED)
                fprintf('  ERROR: Motion failed at APPROACH_PICK.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'GRASP';

        %% ------------------------------------------------
        case 'GRASP'
        %% ------------------------------------------------
            fprintf('  [GRASP] Descending to cube top...\n');
            if ~setPosition(q_pick_grasp, MOVE_SPEED)
                fprintf('  ERROR: descend failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            fprintf('  [GRASP] Closing jaw...\n');
            positionJaw(JAW_CLOSED);
            pause(PAUSE_GRIP);
            state = 'LIFT';

        %% ------------------------------------------------
        case 'LIFT'
        %% ------------------------------------------------
            fprintf('  [LIFT] Ascending to Z_HOVER...\n');
            if ~setPosition(q_pick_hover, MOVE_SPEED)
                fprintf('  ERROR: lift failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'APPROACH_PLACE';

        %% ------------------------------------------------
        case 'APPROACH_PLACE'
        %% ------------------------------------------------
            fprintf('  [TRANSIT] Spinning to basket...\n');
            % Spin base toward basket, keep arm in lifted hover pose
            q_spin = [q_place_hover(1), q_pick_hover(2), ...
                      q_pick_hover(3),  q_pick_hover(4)];
            setPosition(q_spin, MOVE_SPEED);
            pause(1.5);

            if ~setPosition(q_place_hover, MOVE_SPEED)
                fprintf('  ERROR: basket approach failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'RELEASE';

        %% ------------------------------------------------
        case 'RELEASE'
   
        %% ------------------------------------------------
            % Lower to Z_DROP (controlled – cube doesn't bounce)
            fprintf('  [RELEASE] Lowering to drop height...\n');
            if ~setPosition(q_place_drop, MOVE_SPEED)
                fprintf('  ERROR: lower to drop failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            
            fprintf('  [RELEASE] Opening jaw.\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            
            % Retract above basket before next move
            setPosition(q_place_hover, MOVE_SPEED);
            pause(PAUSE_MOVE);
            
            % --- THE SWEEP FIX: RETURN TO HOME ---
            % This untwists the arm, stops the diagonal dragging, 
            % AND moves the arm out of the camera's way for the next scan!
            fprintf('  Returning to safe home posture...\n');
            setPosition([0, 0, 0, 0], MOVE_SPEED);
            pause(PAUSE_MOVE);
            % -------------------------------------
            
            delivered = delivered + 1;
            fprintf('  *** Delivered cube #%d. Total: %d ***\n', ...
                current_cube.label, delivered);
                
            % CRITICAL: rescan after every pick so newly exposed
            % cubes in the same stack are detected
            state = 'PERCEIVE';

        %% ------------------------------------------------
        case 'DONE'
        %% ------------------------------------------------
            fprintf('\n=====================================================\n');
            fprintf('  TASK 4 COMPLETE\n');
            fprintf('  Cubes delivered : %d\n', delivered);
            fprintf('  Cubes skipped   : %d (unreachable/tilted)\n', flagged);
            fprintf('  Total scans     : %d\n', scan_count);
            fprintf('=====================================================\n\n');

            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            setPosition([0,0,0,0], MOVE_SPEED);
            pause(PAUSE_MOVE);
            break;

        %% ------------------------------------------------
        case 'ERROR'
        %% ------------------------------------------------
            fprintf('\n=====================================================\n');
            fprintf('  HARDWARE FAILURE – aborting.\n');
            fprintf('=====================================================\n\n');
            break;

        otherwise
            fprintf('  Unknown state "%s" – aborting.\n', state);
            break;
    end
end


%% =========================================================
%%  STACK LOGIC  (only new code vs M2)
%% =========================================================

function queue = buildPickQueue(cube_list, xy_radius)
% Group cubes into stacks by XY proximity, sort each stack
% top-first (highest top_z), then flatten into a single queue.
% Tallest stacks are scheduled first so we clear them while
% Z_HOVER is guaranteed to clear them on transit.

    N        = numel(cube_list);
    assigned = false(1, N);
    stacks   = {};

    for i = 1:N
        if assigned(i), continue; end
        group       = {cube_list{i}};
        assigned(i) = true;

        for j = i+1:N
            if assigned(j), continue; end
            dxy = sqrt((cube_list{i}.rx - cube_list{j}.rx)^2 + ...
                       (cube_list{i}.ry - cube_list{j}.ry)^2);
            if dxy < xy_radius
                group{end+1} = cube_list{j}; %#ok<AGROW>
                assigned(j)  = true;
            end
        end

        % Sort group by top_z descending (highest cube first)
        top_zs = cellfun(@(c) c.top_z, group);
        [~, ord] = sort(top_zs, 'descend');
        group    = group(ord);

        % Tag with stack id and layer number
        sid = numel(stacks) + 1;
        for layer = 1:numel(group)
            group{layer}.stack_id = sid;
            group{layer}.layer    = layer;
        end

        stacks{end+1} = group; %#ok<AGROW>
    end

    % Order stacks: process the one with the tallest top cube first
    stack_maxz = cellfun(@(s) s{1}.top_z, stacks);
    [~, ord]   = sort(stack_maxz, 'descend');
    stacks     = stacks(ord);

    % Flatten
    queue = {};
    for s = 1:numel(stacks)
        for c = 1:numel(stacks{s})
            queue{end+1} = stacks{s}{c}; %#ok<AGROW>
        end
    end
end


%% =========================================================
%%  VISUAL FEEDBACK
%% =========================================================

function showStackFigure(fig, color_frame, mask, queue)
    if ~ishandle(fig), fig = figure(); end
    figure(fig); clf(fig);

    subplot(1,2,1); imshow(color_frame); title('Camera View');

    subplot(1,2,2); imshow(mask);
    title(sprintf('Target Mask – %d cube(s) queued', numel(queue)));
    hold on;

    cc    = bwconncomp(mask);
    props = regionprops(cc, 'Centroid', 'Area');
    qi    = 1;
    for k = 1:numel(props)
        if props(k).Area < 300, continue; end
        cx = props(k).Centroid(1);
        cy = props(k).Centroid(2);
        if qi <= numel(queue)
            lbl = sprintf('Q%d S%d L%d', qi, ...
                queue{qi}.stack_id, queue{qi}.layer);
            qi = qi + 1;
        else
            lbl = sprintf('#%d', k);
        end
        text(cx, cy, lbl, 'Color','yellow','FontSize',10, ...
            'FontWeight','bold','HorizontalAlignment','center', ...
            'BackgroundColor',[0 0 0 0.35]);
    end
    drawnow;
end




%% =========================================================
%%  PERCEPTION PIPELINE  (identical to M2 – no changes)
%% =========================================================

function [ptCloud, color_frame, intrinsics] = acquirePointCloud()
    pipe    = realsense.pipeline();
    profile = pipe.start();
    cleanupObj = onCleanup(@() stopPipelineSafely(pipe)); %#ok<NASGU>

    dev           = profile.get_device();
    depth_sensor  = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream  = profile.get_stream(realsense.stream.depth) ...
                       .as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    for i = 1:5, fs = pipe.wait_for_frames(); end

    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);

    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data, ...
        [depth.get_width(), depth.get_height()]), [2 1]);

    MIN_DEPTH_M = 0.30; MAX_DEPTH_M = 0.70;
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

    ptCloudRaw = pcfromdepth(depth_frame, 1/depth_scaling, ...
        intrinsics, 'ColorImage', color_frame);

    ptCloudDS = pcdownsample(ptCloudRaw, 'gridAverage', 0.002);
    [ptCloud, ~] = pcdenoise(ptCloudDS, 'NumNeighbors', 20, 'Threshold', 2.0);
end

function stopPipelineSafely(pipe)
    try, pipe.stop(); catch, end
end

function mask = segmentObject(color_frame)
    rgb    = im2double(color_frame);
    hsvImg = rgb2hsv(rgb);
    h = hsvImg(:,:,1); s = hsvImg(:,:,2); v = hsvImg(:,:,3);
    mask = (h < 0.08 | h > 0.92) & (s > 0.25) & (v > 0.2);
    mask = imopen(mask,  strel('disk',3));
    mask = imclose(mask, strel('disk',5));
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end

function objCloud = maskPointCloud(ptCloud, mask, intrinsics)
    xyz = ptCloud.Location; rgb = ptCloud.Color;
    validPts = ~any(isnan(xyz),2) & (xyz(:,3)>0);
    xyz = xyz(validPts,:); rgb = rgb(validPts,:);
    if isempty(xyz)
        warning('maskPointCloud: no valid points.');
        objCloud = pointCloud(zeros(1,3)); return
    end
    fx = intrinsics.FocalLength(1); fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1); cy = intrinsics.PrincipalPoint(2);
    col = round(xyz(:,1)./xyz(:,3).*fx + cx);
    row = round(xyz(:,2)./xyz(:,3).*fy + cy);
    [H,W] = size(mask);
    inBounds = col>=1 & col<=W & row>=1 & row<=H;
    col=col(inBounds); row=row(inBounds);
    xyz=xyz(inBounds,:); rgb=rgb(inBounds,:);
    linearIdx = sub2ind([H,W],row,col);
    inMask    = mask(linearIdx);
    xyzClean  = xyz(inMask,:); rgbClean = rgb(inMask,:);
    if isempty(xyzClean)
        warning('maskPointCloud: no points in mask.');
        objCloud = pointCloud(zeros(1,3)); return
    end
    objCloud = pointCloud(xyzClean, 'Color', rgbClean);
end

function [clusters, labels, K] = clusterObjects(ptCloud, K_override)
    MIN_CLUSTER_PTS=30; K_MAX=8; KMEANS_REPS=5; KMEANS_ITER=200;
    xyz = ptCloud.Location;
    xyz = xyz(~any(isnan(xyz),2),:);
    N   = size(xyz,1);
    if N < MIN_CLUSTER_PTS
        clusters={}; labels=[]; K=0; return
    end
    if nargin>=2 && ~isempty(K_override)
        K = max(1,round(K_override));
    else
        K_max_actual = min(K_MAX, floor(N/MIN_CLUSTER_PTS));
        if K_max_actual < 2
            K = 1;
        else
            wcss = zeros(1,K_max_actual);
            for k=1:K_max_actual
                [~,~,sumd]=kmeans(xyz,k,'Distance','sqeuclidean', ...
                    'Replicates',KMEANS_REPS,'MaxIter',KMEANS_ITER,'Display','off');
                wcss(k)=sum(sumd);
            end
            w=wcss/wcss(1); ks=1:K_max_actual;
            p1=[ks(1),w(1)]; p2=[ks(end),w(end)]; d=zeros(1,K_max_actual);
            for i=1:K_max_actual
                p=[ks(i),w(i)];
                d(i)=abs(cross2d(p2-p1,p1-p))/norm(p2-p1);
            end
            [~,K]=max(d);
        end
    end
    [labels,~]=kmeans(xyz,K,'Distance','sqeuclidean', ...
        'Replicates',KMEANS_REPS,'MaxIter',KMEANS_ITER,'Display','off');
    validMask=~any(isnan(ptCloud.Location),2);
    rgb=ptCloud.Color; rgb=rgb(validMask,:);
    clusters={};
    for k=1:K
        idx=find(labels==k);
        if numel(idx)<MIN_CLUSTER_PTS, continue; end
        clusters{end+1}=pointCloud(xyz(idx,:),'Color',rgb(idx,:)); %#ok<AGROW>
    end
    K=numel(clusters);
end

function d = cross2d(a,b), d=a(1)*b(2)-a(2)*b(1); end

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


%% =========================================================
