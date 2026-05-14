%% =========================================================
% Task 3 – Two-Bin Color Sorting  (20 points)
%
% OVERVIEW
%   8 boxes (cubes + rectangular) are placed randomly.
%   4 are Color-1 or Color-2 (targets); 4 are distractors.
%   The robot must:
%     • Sort Color-1 boxes → Bin 1
%     • Sort Color-2 boxes → Bin 2
%     • Ignore all other colours (no penalty picks)
%     • PLACE boxes (gripper lowered, released at rest) – not drop
%     • Detect rectangular geometry and adapt grasp to short axis
%     • Plan bin placement so boxes don't pile on each other
%
% FSM STATES
%   INIT           – open jaw, home arm
%   PERCEIVE       – run dual-colour perception, build sorted queue
%   NEXT_TARGET    – pop next box from queue
%   CLASSIFY_SHAPE – cube vs. rectangle + compute grasp axis
%   PLAN           – IK for pick + place in correct bin slot
%   APPROACH_PICK  – base spin → hover above box
%   GRASP          – descend with correct wrist angle, close jaw
%   LIFT           – ascend to hover
%   APPROACH_PLACE – transit to bin hover
%   PLACE          – descend into bin slot, open jaw (PLACED not dropped)
%   NEXT_TARGET    – loop until queue empty
%   DONE           – completion signal
%   ERROR          – hardware fault
%
% RECTANGULAR BOX DETECTION
%   PCA of the cluster gives three eigenvectors sorted by variance.
%   For a cube  : all three extents are roughly equal → aspect_ratio ≈ 1
%   For a rect  : one extent is much longer → aspect_ratio > RECT_THRESH
%   When a rectangle is detected the gripper yaw (base joint θ1) is rotated
%   by the in-plane PCA angle so the jaw aligns with the SHORT axis.
%
% BIN PLACEMENT PLANNING
%   Each bin has a 2-D grid of slots.  A slot counter tracks the next
%   empty position so boxes are placed side-by-side, not stacked.
% =========================================================

%% =====================  USER CONFIG  =====================

% --- Color 1 (e.g. RED) ---
COLOR1_NAME = 'Red';
% HSV thresholds for Color 1  [H_lo H_hi S_min V_min]
C1_H_LO = 0.00;  C1_H_HI = 0.08;   % red wraps: also check > 0.92
C1_H_HI2 = 0.92;                    % upper-wrap hue
C1_S_MIN = 0.25;  C1_V_MIN = 0.20;

% --- Color 2 (e.g. BLUE) ---
% --- Color 2 (e.g. BLUE) ---
COLOR2_NAME = 'Blue';

% Widened Hue to catch teal/light blue variations
C2_H_LO  = 0.48;  C2_H_HI  = 0.75;  
% Lowered Saturation to stop lab glare from erasing the blocks!
C2_S_MIN = 0.20;  C2_V_MIN = 0.20;

% --- Bin locations (mm, robot base frame, bin CENTRE) ---
% --- Bin locations (mm, robot base frame, bin CENTRE) ---
BIN1_X   =  120;   BIN1_Y   =  80;    % Moved closer!
BIN2_X   = -120;   BIN2_Y   =  80;    % Moved closer!

% Bin grid: boxes placed in a 2-col × N-row grid inside each bin
SLOT_DX  =   45;   
SLOT_DY  =   45;   
BIN_COLS =    2;   

% --- Heights (mm) ---
Z_HOVER  =  100;   % Lowered from 130 to give the arm more horizontal stretch!
Z_GRASP  =   48;   
Z_PLACE  =   48;

% --- Gripper ---
JAW_OPEN       =  34;   % mm – fully open
JAW_CUBE       =  23;   % mm – closing width for a square cube
JAW_RECT_SHORT =  20;   % mm – closing width for rect box short axis (tune)

% --- Geometry classifier ---
RECT_ASPECT_THRESH = 1.6;  % eigen-length ratio above which → rectangular box
                            % (1.0 = perfect cube; raise if cubes get mis-classified)

% --- Kinematics ---
PHI        = -pi/2;   % end-effector pitch (gripper pointing straight down)
MOVE_SPEED =  50;
PAUSE_MOVE =   2.5;
PAUSE_GRIP =   2.0;



% =====================  END CONFIG  =======================

global arb;

fprintf('\n=====================================================\n');
fprintf('  Task 3 – Two-Bin Color Sorting\n');
fprintf('  Color 1 (%s) → Bin at [%.0f, %.0f] mm\n', ...
    COLOR1_NAME, BIN1_X, BIN1_Y);
fprintf('  Color 2 (%s) → Bin at [%.0f, %.0f] mm\n', ...
    COLOR2_NAME, BIN2_X, BIN2_Y);
fprintf('=====================================================\n\n');

%% =========================================================
%  FSM  –  outer loop
% =========================================================
state = 'INIT';

% Work queues – each entry: struct with fields
%   .position  [x y z] mm in robot frame
%   .rotation  3×3 PCA matrix
%   .color_id  1 or 2
%   .label     integer for display
%   .is_rect   logical
%   .grasp_yaw radians – wrist/base correction for rect boxes
box_queue       = {};
current_box     = [];
delivered_c1    = 0;
delivered_c2    = 0;
skipped_count   = 0;

% Bin slot counters (incremented each time a box is placed)
bin1_slot = 0;
bin2_slot = 0;

% IK waypoints (recomputed each PLAN)
q_pick_hover  = [];
q_pick_grasp  = [];
q_place_hover = [];
q_place_grasp = [];
jaw_width     = JAW_CUBE;

fig = figure('Name', 'Task 3 – Two-Bin Sorting', 'NumberTitle', 'off');

while true
    fprintf('[FSM] State: %s\n', state);

    switch state

        %% ------------------------------------------------
        case 'INIT'
        %% ------------------------------------------------
            fprintf('  Opening jaw and homing arm...\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);

            if ~setPosition([0, 0, 0, 0], MOVE_SPEED)
                fprintf('  WARNING: Could not reach home.\n');
            end
            pause(PAUSE_MOVE);

            state = 'PERCEIVE';

        %% ------------------------------------------------
        case 'PERCEIVE'
        %% ------------------------------------------------
            fprintf('  Capturing scene and classifying colours...\n');

            [ptCloud, color_frame, intrinsics] = acquirePointCloud();

            % --- Segment BOTH colours independently ---
            mask1 = segmentColor(color_frame, 1, ...
                C1_H_LO, C1_H_HI, C1_H_HI2, C1_S_MIN, C1_V_MIN, true);
            mask2 = segmentColor(color_frame, 2, ...
                C2_H_LO, C2_H_HI, 0,         C2_S_MIN, C2_V_MIN, false);

            box_queue = {};
            label_idx = 0;

            for color_id = 1:2
                if color_id == 1
                    mask_c = mask1;
                    cname  = COLOR1_NAME;
                else
                    mask_c = mask2;
                    cname  = COLOR2_NAME;
                end

                objCloud_c = maskPointCloud(ptCloud, mask_c, intrinsics);

                if objCloud_c.Count < 30
                    fprintf('  No %s objects found.\n', cname);
                    continue
                end

                [clusters_c, ~, Kc] = clusterObjects(objCloud_c);
                fprintf('  %s: %d cluster(s) found.\n', cname, Kc);

                if isempty(clusters_c), continue; end

                poses_c = estimatePose(clusters_c);

                for k = 1:length(poses_c)
                    if any(isnan(poses_c{k}.position)), continue; end
                    label_idx = label_idx + 1;

                    entry.position  = double(poses_c{k}.position * 1000); % → mm
                    entry.rotation  = poses_c{k}.rotation;
                    entry.extents   = poses_c{k}.extents;   % [e1 e2 e3] sorted desc
                    entry.color_id  = color_id;
                    entry.label     = label_idx;
                    entry.is_rect   = false;   % filled in CLASSIFY_SHAPE
                    entry.grasp_yaw = 0;
                    % Z override – cube rests on table, so set to half-height
                    % For a rect box this is also the physical half-height:
                    entry.position(3) = max(entry.extents(3), 20) * 500;
                    % (extents are in metres; *500 = half-height in mm)
                    % Clamp to reasonable range
                    entry.position(3) = min(max(entry.position(3), 20), 60);

                    box_queue{end+1} = entry; %#ok<AGROW>
                end
            end

            fprintf('  Total boxes in queue: %d\n', numel(box_queue));
            showSortingFigure(fig, color_frame, mask1, mask2, box_queue, ...
                COLOR1_NAME, COLOR2_NAME);

            if isempty(box_queue)
                state = 'DONE';
            else
                state = 'NEXT_TARGET';
            end

        %% ------------------------------------------------
        case 'NEXT_TARGET'
        %% ------------------------------------------------
            if isempty(box_queue)
                state = 'DONE';
                continue
            end

            current_box = box_queue{1};
            box_queue   = box_queue(2:end);

            cname = COLOR1_NAME;
            if current_box.color_id == 2, cname = COLOR2_NAME; end

            fprintf('\n  === Box #%d | Colour: %s | Pos: [%.0f, %.0f, %.0f] mm ===\n', ...
                current_box.label, cname, ...
                current_box.position(1), current_box.position(2), current_box.position(3));

            state = 'CLASSIFY_SHAPE';

        %% ------------------------------------------------
        case 'CLASSIFY_SHAPE'
        %% ------------------------------------------------
            % PCA extents: [largest, medium, smallest] in metres
            e = current_box.extents;    % [e1 e2 e3]  e1 >= e2 >= e3

            aspect = e(1) / max(e(2), 1e-6);   % ratio of longest to second axis

            if aspect >= RECT_ASPECT_THRESH
                current_box.is_rect = true;
                jaw_width = JAW_RECT_SHORT;

                % The LONG axis of the box in the XY plane is PCA column 1.
                % The gripper must align with the SHORT axis (PCA col 2).
                % We extract the in-plane angle of the SHORT axis:
                short_axis_xy = current_box.rotation(1:2, 2); % [x; y] component
                grasp_yaw     = atan2(short_axis_xy(2), short_axis_xy(1));
                current_box.grasp_yaw = grasp_yaw;

                fprintf('  Shape: RECTANGULAR  aspect=%.2f  short-axis yaw=%.1f deg\n', ...
                    aspect, rad2deg(grasp_yaw));
            else
                current_box.is_rect   = false;
                current_box.grasp_yaw = 0;
                jaw_width             = JAW_CUBE;
                fprintf('  Shape: CUBE  aspect=%.2f\n', aspect);
            end

            state = 'PLAN';

        %% ------------------------------------------------
      
            %% ------------------------------------------------
        case 'PLAN'
        %% ------------------------------------------------
            % --- Camera → Robot frame ---
            % Wrapped in double() and kept your custom offsets!
            cx = double(-current_box.position(2) - 2);
            cy = double(-current_box.position(1) + 25);
            cz = double(current_box.position(3)); 
            
            % --- THE TELEPORT BUG FIX ---
            % We completely DELETED the rectangular grasp_yaw math here. 
            % The robot cannot rotate its wrist, so we must command it to 
            % reach for the exact cx and cy coordinates!
            
            % --- Tilt check (DISABLED DUE TO CAMERA NOISE) ---
            % cube_up  = current_box.rotation(:, 3);
            % tilt_deg = acosd(abs(dot(cube_up, [0;0;1])));
            % if tilt_deg > 35
            %     fprintf('  *** UNGRASPABLE: tilt=%.1f deg. Skipping. ***\n', tilt_deg);
            %     skipped_count = skipped_count + 1;
            %     state = 'NEXT_TARGET';
            %     continue
            % end
            
            % --- IK: pick ---
            current_servo = double([arb.getpos(1), arb.getpos(2), ...
                             arb.getpos(3), arb.getpos(4)]);
            current_dh    = servo2dh(current_servo);
            
            % WE PASS cx AND cy DIRECTLY! 
            q_pick_hover = my_findSolution(cx, cy, Z_HOVER, PHI, current_dh);
            if isempty(q_pick_hover)
                fprintf('  *** UNGRASPABLE: no pick-hover IK. Skipping. ***\n');
                skipped_count = skipped_count + 1;
                state = 'NEXT_TARGET'; continue
            end
            
            q_pick_grasp = my_findSolution(cx, cy, Z_GRASP, PHI, q_pick_hover);
            if isempty(q_pick_grasp)
                fprintf('  *** UNGRASPABLE: no pick-grasp IK. Skipping. ***\n');
                skipped_count = skipped_count + 1;
                state = 'NEXT_TARGET'; continue
            end
            
            % --- IK: place (bin slot) ---
            [slot_x, slot_y] = getBinSlot( ...
                current_box.color_id, ...
                BIN1_X, BIN1_Y, BIN2_X, BIN2_Y, ...
                bin1_slot, bin2_slot, ...
                SLOT_DX, SLOT_DY, BIN_COLS);
                
            q_place_hover = my_findSolution(slot_x, slot_y, Z_HOVER, PHI, q_pick_hover);
            if isempty(q_place_hover)
                fprintf('  ERROR: No place-hover IK. Aborting.\n');
                state = 'ERROR'; continue
            end
            
            q_place_grasp = my_findSolution(slot_x, slot_y, Z_PLACE, PHI, q_place_hover);
            if isempty(q_place_grasp)
                fprintf('  ERROR: No place-grasp IK. Aborting.\n');
                state = 'ERROR'; continue
            end
            
            fprintf('  Plan OK  slot=[%.0f,%.0f]  rect=%d\n', ...
                slot_x, slot_y, current_box.is_rect);
            state = 'APPROACH_PICK';
        %% ------------------------------------------------
        case 'APPROACH_PICK'
        %% ------------------------------------------------
            fprintf('  [APPROACH_PICK] Spinning base, then hover...\n');

            % % Step 1 – base only, arm straight up
            % setPosition([q_pick_hover(1), 0, 0, 0], MOVE_SPEED);
            % pause(1.5);
            % 
            % % Step 2 – full hover position
            % if ~setPosition(q_pick_hover, MOVE_SPEED)
            %     fprintf('  ERROR: hover motion failed.\n');
            %     state = 'ERROR'; continue
            % end
            % pause(PAUSE_MOVE);
            % state = 'GRASP';

            fprintf('  Turning base to face target...\n');
            
            % --- THE SWOOP FIX ---
            % 1. Turn the base (Joint 1) to match the target, but keep 
            % the shoulder, elbow, and wrist perfectly straight up (0,0,0)
            q_intermediate = [q_pick_hover(1), 0, 0, 0];
            setPosition(q_intermediate, MOVE_SPEED);
            pause(1.5); % Give the base time to spin
            
            fprintf('  Dropping to hover above cube...\n');
            % 2. Now bring the arm straight down to the hover position
            if ~setPosition(q_pick_hover, MOVE_SPEED)
                fprintf('  ERROR: Motion failed at APPROACH_PICK.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'GRASP';

        %% ------------------------------------------------
        case 'GRASP'
        %% ------------------------------------------------
            fprintf('  [GRASP] Descending...\n');
            if ~setPosition(q_pick_grasp, MOVE_SPEED)
                fprintf('  ERROR: descend motion failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            fprintf('  [GRASP] Closing jaw to %.1f mm...\n', jaw_width);
            positionJaw(jaw_width);
            pause(PAUSE_GRIP);
            state = 'LIFT';

        %% ------------------------------------------------
        case 'LIFT'
        %% ------------------------------------------------
            fprintf('  [LIFT] Ascending to hover...\n');
            if ~setPosition(q_pick_hover, MOVE_SPEED)
                fprintf('  ERROR: lift motion failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'APPROACH_PLACE';

        %% ------------------------------------------------
        case 'APPROACH_PLACE'
        %% ------------------------------------------------
            fprintf('  [APPROACH_PLACE] Transit to bin...\n');

            % Step 1 – spin base, keep arm lifted
            q_spin = [q_place_hover(1), q_pick_hover(2), ...
                      q_pick_hover(3),  q_pick_hover(4)];
            setPosition(q_spin, MOVE_SPEED);
            pause(1.5);

            % Step 2 – extend to bin hover
            if ~setPosition(q_place_hover, MOVE_SPEED)
                fprintf('  ERROR: approach-place motion failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);
            state = 'PLACE';

        %% ------------------------------------------------
        case 'PLACE'
        %% ------------------------------------------------
            % PLACE = gripper lowered to Z_PLACE, released at rest
            % NOT dropped from above the rim (scoring penalty avoided)
            fprintf('  [PLACE] Lowering into bin slot...\n');
            if ~setPosition(q_place_grasp, MOVE_SPEED)
                fprintf('  ERROR: place-grasp motion failed.\n');
                state = 'ERROR'; continue
            end
            pause(PAUSE_MOVE);

            fprintf('  [PLACE] Opening jaw – box placed at rest.\n');
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);

            % Retract to hover before next move
            setPosition(q_place_hover, MOVE_SPEED);
            pause(PAUSE_MOVE);

            % Advance bin slot counter
            if current_box.color_id == 1
                bin1_slot    = bin1_slot + 1;
                delivered_c1 = delivered_c1 + 1;
                fprintf('  *** Placed in Bin 1 (%s). Total: %d ***\n', ...
                    COLOR1_NAME, delivered_c1);
            else
                bin2_slot    = bin2_slot + 1;
                delivered_c2 = delivered_c2 + 1;
                fprintf('  *** Placed in Bin 2 (%s). Total: %d ***\n', ...
                    COLOR2_NAME, delivered_c2);
            end

            state = 'NEXT_TARGET';

        %% ------------------------------------------------
        case 'DONE'
        %% ------------------------------------------------
            fprintf('\n=====================================================\n');
            fprintf('  TASK 3 COMPLETE\n');
            fprintf('  Bin 1 (%s) deliveries : %d\n', COLOR1_NAME, delivered_c1);
            fprintf('  Bin 2 (%s) deliveries : %d\n', COLOR2_NAME, delivered_c2);
            fprintf('  Skipped (ungraspable) : %d\n', skipped_count);
            fprintf('=====================================================\n\n');

            % Return to home
            positionJaw(JAW_OPEN);
            pause(PAUSE_GRIP);
            setPosition([0, 0, 0, 0], MOVE_SPEED);
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
%%  BIN SLOT PLANNER
%% =========================================================
function [sx, sy] = getBinSlot(color_id, ...
        bx1, by1, bx2, by2, ...
        slot1, slot2, dx, dy, ncols)
% Returns the (x,y) centre of the next empty slot in the bin.
% Slots are arranged in a grid: row = floor(slot/ncols), col = mod(slot,ncols)

    if color_id == 1
        bx   = bx1;  by = by1;
        slot = slot1;
    else
        bx   = bx2;  by = by2;
        slot = slot2;
    end

    col = mod(slot, ncols);          % 0-indexed column
    row = floor(slot / ncols);       % 0-indexed row

    % Centre grid around bin centre
    half_w = (ncols - 1) / 2 * dx;
    sx = bx + col * dx - half_w;
    sy = by + row * dy;
end


%% =========================================================
%%  VISUAL FEEDBACK
%% =========================================================
function showSortingFigure(fig, color_frame, mask1, mask2, queue, cname1, cname2)

    if ~ishandle(fig), fig = figure(); end
    figure(fig); clf(fig);

    subplot(1, 3, 1);
    imshow(color_frame);
    title('Camera View');

    subplot(1, 3, 2);
    imshow(mask1);
    title(sprintf('%s Mask', cname1));
    hold on;
    labelMaskCentroids(mask1, 'yellow');

    subplot(1, 3, 3);
    imshow(mask2);
    title(sprintf('%s Mask', cname2));
    hold on;
    labelMaskCentroids(mask2, 'cyan');

    drawnow;
end

function labelMaskCentroids(mask, clr)
    cc    = bwconncomp(mask);
    props = regionprops(cc, 'Centroid', 'Area');
    for k = 1:numel(props)
        if props(k).Area < 300, continue; end
        text(props(k).Centroid(1), props(k).Centroid(2), ...
            sprintf('#%d', k), 'Color', clr, 'FontSize', 13, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
end

function flagBoxOnFigure(fig, lbl, reason)
    if ~ishandle(fig), return; end
    figure(fig);
    for sp = 2:3
        subplot(1, 3, sp); hold on;
        xl = xlim(); yl = ylim();
        text(mean(xl), yl(1) + 25*lbl, ...
            sprintf('*** #%d SKIP: %s ***', lbl, reason), ...
            'Color', 'red', 'FontSize', 11, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
    end
    fprintf('  [FLAG] Box #%d marked on screen: %s\n', lbl, reason);
    drawnow;
end



%% =========================================================
%%  PERCEPTION PIPELINE
%% =========================================================
function [ptCloud, color_frame, intrinsics] = acquirePointCloud()
    pipe    = realsense.pipeline();
    profile = pipe.start();
    cleanup = onCleanup(@() safestop(pipe)); %#ok<NASGU>

    dev           = profile.get_device();
    depth_sensor  = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream  = profile.get_stream(realsense.stream.depth) ...
                       .as('video_stream_profile');
    depth_int     = depth_stream.get_intrinsics();

    for i = 1:5, fs = pipe.wait_for_frames(); end

    align = realsense.align(realsense.stream.depth);
    fs    = align.process(fs);

    depth      = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data, ...
        [depth.get_width(), depth.get_height()]), [2 1]);

    MIN_D = 0.30; MAX_D = 0.70;
    dm    = depth_frame .* depth_scaling;
    depth_frame(dm < MIN_D | dm > MAX_D) = 0;

    color      = fs.get_color_frame();
    color_data = color.get_data();
    color_frame = permute(reshape(color_data, ...
        [3, color.get_width(), color.get_height()]), [3 2 1]);

    intrinsics = cameraIntrinsics( ...
        [depth_int.fx, depth_int.fy], ...
        [depth_int.ppx, depth_int.ppy], ...
        size(depth_frame));

    raw  = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ...
               'ColorImage', color_frame);
    ds   = pcdownsample(raw, 'gridAverage', 0.002);
    [ptCloud, ~] = pcdenoise(ds, 'NumNeighbors', 20, 'Threshold', 2.0);
end

function safestop(pipe)
    try, pipe.stop(); catch, end
end

% ---------------------------------------------------------
function mask = segmentColor(color_frame, color_id, ...
        h_lo, h_hi, h_hi2, s_min, v_min, wrap_hue)
% Generic HSV segmenter.
% wrap_hue=true  → red-style double band (h<h_hi OR h>h_hi2)
% wrap_hue=false → single band (h_lo < h < h_hi)

    rgb    = im2double(color_frame);
    hsv    = rgb2hsv(rgb);
    h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);

    if wrap_hue
        hue_ok = (h < h_hi) | (h > h_hi2);
    else
        hue_ok = (h >= h_lo) & (h <= h_hi);
    end

    mask = hue_ok & (s > s_min) & (v > v_min);
    mask = imopen(mask,  strel('disk', 3));
    mask = imclose(mask, strel('disk', 5));
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 400);
end

% ---------------------------------------------------------
function objCloud = maskPointCloud(ptCloud, mask, intrinsics)
    xyz = ptCloud.Location;
    rgb = ptCloud.Color;

    ok  = ~any(isnan(xyz),2) & (xyz(:,3) > 0);
    xyz = xyz(ok,:); rgb = rgb(ok,:);

    if isempty(xyz)
        objCloud = pointCloud(zeros(1,3)); return
    end

    fx = intrinsics.FocalLength(1);  fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1); cy = intrinsics.PrincipalPoint(2);

    col = round(xyz(:,1)./xyz(:,3).*fx + cx);
    row = round(xyz(:,2)./xyz(:,3).*fy + cy);
    [H,W] = size(mask);
    ib  = col>=1 & col<=W & row>=1 & row<=H;
    col = col(ib); row = row(ib);
    xyz = xyz(ib,:); rgb = rgb(ib,:);

    idx = sub2ind([H,W], row, col);
    in  = mask(idx);
    xc  = xyz(in,:); rc = rgb(in,:);

    if isempty(xc)
        objCloud = pointCloud(zeros(1,3)); return
    end
    objCloud = pointCloud(xc, 'Color', rc);
end

% ---------------------------------------------------------
function [clusters, labels, K] = clusterObjects(ptCloud, K_override)
    MIN_PTS  = 30;
    K_MAX    = 8;
    REPS     = 5;
    ITER     = 200;

    xyz = ptCloud.Location;
    xyz = xyz(~any(isnan(xyz),2),:);
    N   = size(xyz,1);

    if N < MIN_PTS
        clusters = {}; labels = []; K = 0; return
    end

    if nargin >= 2 && ~isempty(K_override)
        K = max(1, round(K_override));
    else
        Km = min(K_MAX, floor(N/MIN_PTS));
        if Km < 2
            K = 1;
        else
            wcss = zeros(1,Km);
            for k = 1:Km
                [~,~,sd] = kmeans(xyz,k,'Distance','sqeuclidean', ...
                    'Replicates',REPS,'MaxIter',ITER,'Display','off');
                wcss(k) = sum(sd);
            end
            w  = wcss/wcss(1); ks = 1:Km;
            p1 = [ks(1),w(1)]; p2 = [ks(end),w(end)];
            d  = zeros(1,Km);
            for i = 1:Km
                p    = [ks(i),w(i)];
                d(i) = abs(cross2d(p2-p1,p1-p))/norm(p2-p1);
            end
            [~,K] = max(d);
        end
    end

    [labels,~] = kmeans(xyz,K,'Distance','sqeuclidean', ...
        'Replicates',REPS,'MaxIter',ITER,'Display','off');

    vm  = ~any(isnan(ptCloud.Location),2);
    rgb = ptCloud.Color; rgb = rgb(vm,:);

    clusters = {};
    for k = 1:K
        idx = find(labels==k);
        if numel(idx) < MIN_PTS, continue; end
        clusters{end+1} = pointCloud(xyz(idx,:),'Color',rgb(idx,:)); %#ok<AGROW>
    end
    K = numel(clusters);
end

function d = cross2d(a,b), d = a(1)*b(2)-a(2)*b(1); end

% ---------------------------------------------------------
function poses = estimatePose(clusters)
% Returns poses with .position, .rotation, .extents
% .extents = [e1 e2 e3] half-lengths of PCA bounding box, descending.

    poses = cell(1, length(clusters));
    for k = 1:length(clusters)
        xyz = clusters{k}.Location;
        xyz = xyz(~any(isnan(xyz),2),:);

        if size(xyz,1) < 3
            poses{k}.position = [NaN NaN NaN];
            poses{k}.rotation = eye(3);
            poses{k}.extents  = [0 0 0];
            continue
        end

        mu  = mean(xyz,1);
        [R, ~, ~] = pca(xyz);

        % Project onto PCA axes to get bounding-box half-lengths
        proj = (xyz - mu) * R;   % N×3, each column is one PCA axis
        half = (max(proj,1) - min(proj,1)) / 2;  % [e1 e2 e3]
        half = sort(half, 'descend');

        poses{k}.position = mu;
        poses{k}.rotation = R;
        poses{k}.extents  = half;   % metres (point cloud is in metres)
    end
end