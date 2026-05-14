function clusters = clusterObjects(ptCloud)

    if ptCloud.Count == 0
        clusters = {};
        return;
    end

    labels = pcsegdist(ptCloud, 0.02);

    if isempty(labels)
        clusters = {};
        return;
    end

    n = max(labels);
    clusters = cell(1,n);

    for k = 1:n
        clusters{k} = select(ptCloud, labels==k);
    end
end
