function [ptCloud, color_frame, intrinsics] = acquirePointCloud()

    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();

    %% Acquire device parameters 
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();

    % Extract the depth stream and intrinsics
    depth_stream = profile.get_stream(realsense.stream.depth) ...
                           .as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    %% Acquire frames (discard first few)
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    
    % Align frames
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % Stop streaming
    pipe.stop();

    %% Extract depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute( ...
        reshape(depth_data', [depth.get_width(), depth.get_height()]), ...
        [2 1]);

    %% Extract color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute( ...
        reshape(color_data', [3, color.get_width(), color.get_height()]), ...
        [3 2 1]);

    %% Camera intrinsics (MATLAB format)
    intrinsics = cameraIntrinsics( ...
        [depth_intrinsics.fx, depth_intrinsics.fy], ...
        [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
        size(depth_frame));

    %% Create point cloud
    ptCloud = pcfromdepth( ...
        depth_frame, ...
        1/depth_scaling, ...
        intrinsics, ...
        ColorImage=color_frame);

end
