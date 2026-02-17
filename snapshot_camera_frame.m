function snapshot_camera_frame()
    % 1. Start Pipeline
    pipe = realsense.pipeline();
    profile = pipe.start();

    % 2. Warm-up (Standard RealSense practice for auto-exposure)
    for i = 1:10
        fs = pipe.wait_for_frames();
    end

    % 3. Capture the Single Snapshot
    fs = pipe.wait_for_frames();
    color = fs.get_color_frame();
    
    % Stop the pipeline immediately after capture
    pipe.stop();

    % 4. Process Image Data
    data = color.get_data();
    w = color.get_width();
    h = color.get_height();
    % Reshape and permute to MATLAB RGB format [H x W x 3]
    color_frame = permute(reshape(data', [3, w, h]), [3, 2, 1]);

    %% --- ASSIGN WORLD/CAMERA FRAME AT IMAGE CENTER ---
    % Define the pixel origin (Center of the image)
    originX = w / 2;
    originY = h / 2;

    % Define axis length in pixels
    scale = 80; 

    % Create the figure
    figure('Name', 'Static World Frame Assignment');
    imshow(color_frame);
    hold on;

    % X-Axis (Red): Arbitrarily assigned to the right (Horizontal)
    quiver(originX, originY, scale, 0, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 1);
    
    % Y-Axis (Green): Arbitrarily assigned downwards (Vertical)
    quiver(originX, originY, 0, scale, 0, 'g', 'LineWidth', 4, 'MaxHeadSize', 1);

    % Z-Axis (Blue): Pointing into the scene (Downwards from camera lens)
    % Represented as a dot/circle because it is perpendicular to the screen
    plot(originX, originY, 'bo', 'MarkerSize', 15, 'LineWidth', 3);
    plot(originX, originY, 'b.', 'MarkerSize', 20);

    % Label the Font Colors (Yellow for visibility)
    text(originX + scale + 10, originY, 'X', 'Color', 'y', 'FontWeight', 'bold', 'FontSize', 16);
    text(originX, originY + scale + 20, 'Y', 'Color', 'y', 'FontWeight', 'bold', 'FontSize', 16);
    text(originX - 40, originY - 30, 'Z (Down)', 'Color', 'y', 'FontWeight', 'bold', 'FontSize', 14);

    title('Single Frame World/Camera Frame Assignment');
    hold off;
end