function objCloud = maskPointCloud(ptCloud, mask)

    maskVector = mask(:); % convert 2D mask to vector
    objCloud = select(ptCloud, find(maskVector));

end
