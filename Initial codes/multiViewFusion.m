function ptCloudFused = multiViewFusion(ptClouds)

ptCloudFused = ptClouds{1};
ptCloudRef   = ptCloudFused;

for i = 2:length(ptClouds)

    tform = pcregistericp( ...
        ptClouds{i}, ...
        ptCloudRef, ...
        'Metric','pointToPlane', ...
        'Extrapolate',true);

    ptAligned = pctransform(ptClouds{i},tform);

    ptCloudFused = pcmerge(ptCloudFused,ptAligned,0.005);

    ptCloudRef = ptCloudFused;
end

end
