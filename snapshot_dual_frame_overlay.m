function snapshot_dual_frame_overlay()
    % 1. Startup and Capture (Same as your original code)
    pipe = realsense.pipeline();
    profile = pipe.start();
    
    % Get Parameters & Intrinsics
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    intrinsics = depth_stream.get_intrinsics(); % We need these for projection

    % Warm up and capture
    for i = 1:15, fs = pipe.wait_for_frames(); end
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    pipe.stop();

    % Process frames
    color = fs.get_color_frame();
    color_data = color.get_data();
    w = color.get_width(); h = color.get_height();
    color_frame = permute(reshape(color_data', [3, w, h]), [3, 2, 1]);

    %% --- [INTERNAL MATH: Reuse your MABB/Centroid/Axes Logic here] ---
    % Assume variables 'centroid', 'xAxis', 'yAxis', 'zAxis' are calculated 
    % exactly as they were in your point cloud script.
    
    % Define the 3D tips of the axis arrows (e.g., 5cm long)
    len = 0.05;
    tipX_3d = centroid + xAxis * len;
    tipY_3d = centroid + yAxis * len;
    tipZ_3d = centroid + zAxis * len;

    % Create a matrix of 3D points to project: [Origin, TipX, TipY, TipZ]
    pts3d = [centroid; tipX_3d; tipY_3d; tipZ_3d];

    %% --- 3D TO 2D PROJECTION ---
    % Project 3D points to 2D pixel coordinates using intrinsics
    % u = (x * fx / z) + ppx ; v = (y * fy / z) + ppy
    pts2d = zeros(4, 2);
    for i = 1:4
        pts2d(i, 1) = (pts3d(i, 1) * intrinsics.fx / pts3d(i, 3)) + intrinsics.ppx;
        pts2d(i, 2) = (pts3d(i, 2) * intrinsics.fy / pts3d(i, 3)) + intrinsics.ppy;
    end

    %% --- FINAL VISUALIZATION ---
    figure('Name', 'Dual Frame Overlay');
    imshow(color_frame); hold on;

    % 1. DRAW WORLD/CAMERA FRAME (At image center)
    camOrigin = [w/2, h/2];
    cScale = 60;
    line([camOrigin(1) camOrigin(1)+cScale], [camOrigin(2) camOrigin(2)], 'Color', 'r', 'LineWidth', 2); % X
    line([camOrigin(1) camOrigin(1)], [camOrigin(2) camOrigin(2)+cScale], 'Color', 'g', 'LineWidth', 2); % Y
    plot(camOrigin(1), camOrigin(2), 'bo', 'MarkerSize', 8); % Z
    text(camOrigin(1)+5, camOrigin(2)-10, 'Camera Frame', 'Color', 'y', 'FontSize', 8);

    % 2. DRAW PROJECTED BLOCK FRAME (At object center)
    origin2d = pts2d(1,:);
    x2d = pts2d(2,:);
    y2d = pts2d(3,:);
    z2d = pts2d(4,:);

    % Draw X (Red), Y (Green), Z (Blue)
    line([origin2d(1) x2d(1)], [origin2d(2) x2d(2)], 'Color', 'r', 'LineWidth', 4);
    line([origin2d(1) y2d(1)], [origin2d(2) y2d(2)], 'Color', 'g', 'LineWidth', 4);
    line([origin2d(1) z2d(1)], [origin2d(2) z2d(2)], 'Color', 'b', 'LineWidth', 4);

    % Label the Block Frame
    text(x2d(1), x2d(2), 'X', 'Color', 'y', 'FontWeight', 'bold');
    text(y2d(1), y2d(2), 'Y', 'Color', 'y', 'FontWeight', 'bold');
    text(z2d(1), z2d(2), 'Z', 'Color', 'y', 'FontWeight', 'bold');

    title('RGB Overlay: World Frame (Center) vs Object Frame (Projected)');
    hold off;
end