function MapDensity = MapDensityFun(ptMap)
radius = 1;
ratio = 2000 ./ ptMap.Count;
ptSed = pcdownsample(ptMap, 'random',  ratio);
kdMap = createns(ptMap.Location);
[vvIdx, ~] = rangesearch(kdMap, ptSed.Location, radius);
%
vNum = [];
for n = 1 : 1 : ptSed.Count
    vIdx = vvIdx{n}; 
    vNum = [vNum; length(vIdx)];
end
MapDensity = mean(vNum);
end