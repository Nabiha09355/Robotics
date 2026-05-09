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
    % Look for anything between 40cm and 60cm away from the lens
MIN_DEPTH_M = 0.40; 
MAX_DEPTH_M = 0.60;
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
