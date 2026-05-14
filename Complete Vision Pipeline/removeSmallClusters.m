function cleanCloud = removeSmallClusters(ptCloud, minPoints)

    % Segment into clusters
    labels = pcsegdist(ptCloud, 0.02);

    % If nothing detected
    if isempty(labels)
        cleanCloud = ptCloud;
        return;
    end

    % Count points in each cluster
    counts = histcounts(labels);

    % Keep clusters larger than threshold
    validClusters = find(counts > minPoints);

    % Logical mask
    keep = ismember(labels, validClusters);

    % Select only valid clusters
    cleanCloud = select(ptCloud, keep);

end
