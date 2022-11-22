function params = getKitParamFun(DataSet, MapType, TestName, RandmSparse, NeibK)

params.MapType = MapType; %vMapTypes{1};
params.isShow = 0;
% KitSeq00 Campus Sudoku%
params.isFirst = 1;
params.isGridDown = 0;  % random downsample
params.ptNum = 2000;
params.ScaleC = 100;
params.isRobustLoss = 1;
% Data association method, which can be replaced by NDT,ICP and others
% kNN is more faster than others.
params.DataAssociM = 'kNN';
if strcmp(DataSet, 'KitSeq00')
    params.SeqIdx = 0;
    params.ExpName = 'Kitti';
    params.CalTF = eye(4);
    params.InvCalTF = inv(params.CalTF);
    params.MinR = 5;     	% DemoAdaptiveKit06_2000 2020-04-28
    params.MaxR = 40;
    params.StartID = 1;
    A = load('KITTI00/GrdTruth.mat');    %
    params.dFrm = 1;
    B = load('KITTI00/OdoPose.mat');
    params.CovM = diag(B.OnLineV);  % 2020-05-03  OnLine
    params.vGrdTF = A.vTestTFNew;
    params.vGrdPose = A.vTestPoseNew;
    params.PoseHDL = A.vTestTF;
    params.SelRange = A.vIdxTest;
    params.vImuTF = B.vNewTF;
    params.vImuPose = CvTF2vEul(params.vImuTF); % the input signal is already noise
    params.GpsTf = [params.vGrdTF(:,:,params.StartID); 0 0 0 1];

    params.Len = size(params.vGrdPose, 1);

    params.MapRoot = 'KITTI00';
	FileName = 'Sqe00Map.mat';

    CovMap = load(fullfile(params.MapRoot, FileName));
    params.ptMap = CovMap.vMapInf.ptMap;
    if strcmp(params.MapType, 'PointCov')
        params.Covs = CovMap.vMapInf.Covs;
        params.inCovs = CovMap.vMapInf.inCovs;
    end
    % 2022-09-16
    params.MapPtDensity = MapDensityFun(params.ptMap);
    %
    VelRoot = 'D:\KittiData\data_odometry_velodyne\sequences';
    params.LocRoot = sprintf('%s/%02d/velodyne/', VelRoot, params.SeqIdx);
    params.ptMapShow = pcdownsample(params.ptMap, 'gridAverage', 1.0); % for display
    params.kdMap = createns(params.ptMap.Location);
end
% figure;pcshow(params.ptMapShow);view(2);box on;axis equal;
params.QCov = 0.01;    % after  2022-03-04
params.RadiusRange = [params.MinR, params.MaxR]; % Limit the range of lidar
params.nFrms = length(params.StartID : params.dFrm : length(params.SelRange));
%%
params.SaveFolder = sprintf('%s%s%.4fNebK%dExp', TestName, params.ExpName, RandmSparse,NeibK);
if ~exist(params.SaveFolder)
    mkdir(params.SaveFolder);
end

end