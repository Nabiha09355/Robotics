function clusters = clusterObjects(ptCloud)
    labels = pcsegdist(ptCloud,0.02);
    n = max(labels);

    clusters = cell(1,n);
    for k = 1:n
        clusters{k} = select(ptCloud, labels==k);
    end
end
