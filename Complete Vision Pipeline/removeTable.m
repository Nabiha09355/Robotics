function blockCloud = removeTable(ptCloud)

    % Remove invalid points first
    ptCloud = removeInvalidPoints(ptCloud);

    xyz = ptCloud.Location;

    % Handle both organized and unorganized clouds
    if ndims(xyz) == 3
        zVals = xyz(:,:,3);
        zVals = zVals(:);
    else
        zVals = xyz(:,3);
    end

    % Remove NaNs
    zVals = zVals(~isnan(zVals));

    % Highest Z plane (table)
    tableZ = prctile(zVals, 99.5);

    offset = 0.01;  % 1 cm

    % Now create mask properly
    if ndims(xyz) == 3
        mask = xyz(:,:,3) < (tableZ - offset);
        blockCloud = select(ptCloud, find(mask(:)));
    else
        mask = xyz(:,3) < (tableZ - offset);
        blockCloud = select(ptCloud, find(mask));
    end

end
