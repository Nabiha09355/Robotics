function objCloud = maskPointCloud(ptCloud, mask)
    xyz = ptCloud.Location;
    rgb = ptCloud.Color;

    mask3D = repmat(mask,[1 1 3]);
    xyz(~mask3D) = NaN;

    objCloud = pointCloud(xyz,'Color',rgb);
end
