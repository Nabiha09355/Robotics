function poses = estimatePose(clusters)

for k = 1:length(clusters)

    block = clusters{k};
    xyz = block.Location;
    xyz = xyz(~any(isnan(xyz),2),:);

    centroid = mean(xyz,1);
    [R,~,~] = pca(xyz);

    xyz_centered = xyz - centroid;
    xyz_pca = xyz_centered * R;

    zValues = xyz_pca(:,3);
    zMax = max(zValues);

    topMask = zValues > (zMax - 0.005);
    topPoints = xyz_pca(topMask,:);

    [~, idx] = max(topPoints(:,1) + topPoints(:,2));
    corner_pca = topPoints(idx,:);

    corner_world = corner_pca * R' + centroid;

    poses{k}.position = corner_world;
    poses{k}.rotation = R;

end

end
