function poses = estimatePose(clusters)
    for k = 1:length(clusters)
        xyz = clusters{k}.Location;
        xyz = xyz(~any(isnan(xyz),2),:);

        t = mean(xyz,1);
        [R,~,~] = pca(xyz);

        poses{k}.position = t;
        poses{k}.rotation = R;
    end
end
