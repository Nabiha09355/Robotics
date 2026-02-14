function pointcloud_example()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();


    %% Acquire device parameters 
    % Get streaming device's name
    dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

    % Extract the depth stream
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    
    % Get the intrinsics
    depth_intrinsics = depth_stream.get_intrinsics();

    %% Align the frames and then get the frames
    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    
    % Alignment is necessary as the depth cameras and RGB cameras are
    % physically separated. So, the same (x,y,z) in real world maps to
    % different (u,v) in the depth image and the color images. To build a
    % point cloud we only need depth image, but if we want the color the
    % cloud then we'll need the other image.

    % Since the two images are of different sizes, we can either align the
    % depth to color image, or the color to depth.
    % Change the argument to realsense.stream.color to align to the color
    % image.
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % Stop streaming
    pipe.stop();

    % Extract the depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);

    % Extract the color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    %% Create a point cloud using MATLAB library
    % Create a MATLAB intrinsics object
    intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    
    % Create a point cloud
    ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);
   
    % Display point cloud
    pcshow(ptCloud,'VerticalAxisDir','Down');

    
xLimits = [-0.1 0.1];   % left-right range
yLimits = [0 0.10];   % front-back range

% Keep ALL Z values
zLimits = [0 0.385];

roi = [xLimits yLimits zLimits];

% Get indices inside ROI
indices = findPointsInROI(ptCloud, roi);

% Extract cropped cloud
croppedCloud = select(ptCloud, indices);


% Show result
figure
pcshow(croppedCloud, 'VerticalAxisDir','Down');
title('Point Cloud without Plane');

% --- DENOISE CROPPED CLOUD ---
croppedCloud = pcdenoise(croppedCloud, ...
                         'NumNeighbors', 20, ...
                         'Threshold', 1);

figure
pcshow(croppedCloud, 'VerticalAxisDir','Down');
axis equal
title('Denoised Cropped Cloud');


xyz = croppedCloud.Location;
xyz = reshape(xyz, [], 3);
xyz = xyz(~any(isnan(xyz),2), :);
maxDistance = 0.0015; % 3 mm tolerance
% --- EXTRACT NUMERIC XYZ ---
xyzAll = croppedCloud.Location;
xyzAll = reshape(xyzAll, [], 3);
xyzAll = xyzAll(~any(isnan(xyzAll),2), :);

% --- KEEP ONLY TOP 20% HIGHEST POINTS ---
zThreshold = prctile(xyzAll(:,3), 80);
xyzTopCandidate = xyzAll(xyzAll(:,3) > zThreshold, :);

% Build temporary point cloud from candidates
topCandidateCloud = pointCloud(xyzTopCandidate);

% Now fit plane ONLY to highest region
[modelTop, inliersTop] = pcfitplane(topCandidateCloud, 0.0015);

topCloud = select(topCandidateCloud, inliersTop);


topCloud = select(croppedCloud, inliersTop);


figure
pcshow(topCloud, 'VerticalAxisDir','Down');
axis equal
title('Top surface');
 figure
pcshow(croppedCloud, ...
       'VerticalAxisDir','Down', ...
       'MarkerSize', 60);

axis equal
view(2)
grid on
title('Top View - Enhanced')



% Extract numeric XYZ from topCloud
xyzTop = topCloud.Location;

% Convert from H×W×3 to N×3
xyzTop = reshape(xyzTop, [], 3);

% Remove NaNs
xyzTop = xyzTop(~any(isnan(xyzTop),2), :);

% Compute centroid
centroid = mean(xyzTop);

% Subtract centroid
centered = xyzTop - centroid;


% PCA
[coeff, ~, ~] = pca(centered);
% --- FORCE NORMAL TO POINT UP ---
if coeff(3,3) < 0
    coeff(:,3) = -coeff(:,3);
end


rotated = centered * coeff;
xmin = prctile(rotated(:,1), 3);
xmax = prctile(rotated(:,1), 97);
ymin = prctile(rotated(:,2), 3);
ymax = prctile(rotated(:,2), 97);

% --- BUILD CORNERS IN ROTATED FRAME (Z = 0) ---
corners_rot = [
    xmin ymin 0;
    xmax ymin 0;
    xmax ymax 0;
    xmin ymax 0
];

% Transform back to camera frame
corners_temp = corners_rot * coeff' + centroid;

% --- PROJECT CORNERS ONTO DETECTED PLANE ---
A = modelTop.Parameters(1);
B = modelTop.Parameters(2);
C = modelTop.Parameters(3);
D = modelTop.Parameters(4);

for i = 1:4
    x = corners_temp(i,1);
    y = corners_temp(i,2);
    
    % Plane equation: Ax + By + Cz + D = 0
    z = -(A*x + B*y + D)/C;
    
    corners_temp(i,3) = z;
end

corners = corners_temp;

figure
pcshow(croppedCloud, ...
       'VerticalAxisDir','Down', ...
       'MarkerSize', 60);

axis equal
view(3)
grid on
hold on
plot3(corners(:,1), corners(:,2), corners(:,3), ...
      'ro', 'MarkerSize', 14, 'LineWidth', 3);
hold off
title('Final Detected Corners');





    
    
end

