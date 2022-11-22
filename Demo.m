%-------------------------------------------------------------------------------
%   2022-11-20 Distance-based EKF for Fast and Robust Map-based Localization
%   This demo relies on the 'Seq00' in the KITTI dataset
%   The employed map is already generated and saved as 'KITTI00/Sqe00Map.mat'.
%-------------------------------------------------------------------------------
clc; close all; clear all;
addpath('DistBasedFun');
FixPts = 1.0.*[-120 -80 -10;-120 -80 10;-120 80 -10;-120 80 10;120 -80 -10;120 -80 10;120 80 -10;120 80 10;];
DataSet = 'KitSeq00';%
Index = 0;
TestName = 'ExpOnKitR';
RandomSparse = 0.02; % keep the map density to 4.866
NeibK = 10; % for covariance estimation
vDistType = {'P2D1', 'D2D1A', 'P2P1','P2PL1'}; % map density = 5, K = 30
for distID = 1 : 1 : length(vDistType) % 2022-09-16 again %
    DistType = vDistType{distID};
    vMapTypes = {'Point','PointNormal','PointCov'};
    if strcmp(DistType, 'P2D1') || strcmp(DistType, 'D2D1A')
        MapType = vMapTypes{3};
    else
        MapType = vMapTypes{1};
    end
    %
    params = getKitParamFun(DataSet, MapType, TestName, RandomSparse, NeibK);
    params.DistType = DistType;
    
    params.ScaleC = 2000; % for different distance, the value is different!
    params.RobustF = 'Cauchy';% 'Tukey','Welsch','Geman-McClure',
    if strcmp(params.RobustF, 'RawD')
        params.isRobustLoss = 0;
    else
        params.isRobustLoss = 1;
    end
    %
    SeqLen = params.Len;
    tTime = zeros(SeqLen, 1);
    tRmse = zeros(SeqLen, 1);
    tRatio = zeros(SeqLen, 2);
    tNum = zeros(SeqLen, 1);    
    vX = zeros(SeqLen, 6);
    vCov = zeros(6, 6, SeqLen);
    vehvX = zeros(SeqLen, 6);
    %
    GpsTF = [eul2rotm(rotm2eul(params.GpsTf(1:3,1:3))),params.GpsTf(1:3,end);0 0 0 1];
    % EKF
    Xk_1 = CTF2Pose( GpsTF );    % InitPose   [R,t]       % 2019-09-07
    Ck_1 = diag([0.001 0.001 0.001 0.001 0.001 0.001] .^ 2); % state = [ax ay az x y z]
    %
    params.isFirst = 1;
    for id = params.StartID : params.dFrm : length( params.SelRange ) %  only for kitti 0426
        nFrm = params.SelRange(id) - 1;
        [ptHdl,ptOrg,~] = KitHdlReadRaw(nFrm, params.LocRoot, params.ptNum, params.RadiusRange, ~params.isGridDown); % 0218 random sample
        tic;
        [XbkTF, Xbk, Gx, Gu] = PropagationTF( id, Xk_1, params );
        Cbk = Gx * Ck_1 * Gx' + Gu * params.CovM * Gu';
        %
        pos = XbkTF(1:3,end)';
        vIdx = rangesearch(params.kdMap, pos, params.MaxR+6);
        if strcmp(params.MapType, 'PointCov')
            PtsCovArg.subMap = pointCloud(params.ptMap.Location(vIdx{:},:));
            PtsCovArg.subMap.Normal = params.ptMap.Normal(vIdx{:},:);
            PtsCovArg.subkdMap = createns(PtsCovArg.subMap.Location);
            PtsCovArg.subvCovs = params.Covs(:, :, vIdx{:});
            PtsCovArg.invvCovs = params.inCovs(:, :, vIdx{:});
            if strcmp(params.DistType, 'D2D1A') 
                if strcmp(TestName, 'NebKCovC') || strcmp(TestName, 'LidarScanNumNewVTotal')
                    [Covs, inCovs, ptHdl] = ScansCovNeibK(ptHdl, ptOrg, NeibK); % Ndt  type
                end
                if strcmp(TestName, 'RangeCovC')
                    [Covs, inCovs, ptHdl] = ScansCovRange(ptHdl, ptOrg, Radius); % Ndt  type
                end
                if strcmp(TestName, 'VoxelGrid')
                    [Covs, inCovs, ptHdl] = ScansCovVoxelGrid(ptHdl, ptOrg, GridSize); % Ndt  type
                end
                PtsCovArg.ptHdlCovs = Covs;
            else
                PtsCovArg.ptHdlCovs = zeros(3, 3, ptHdl.Count);
            end
            params.CorresTF = XbkTF; % knn
            [Xk, Ck] = UpdataGeneCov(PtsCovArg, ptHdl, Xbk, Cbk, XbkTF, params);
        else
            subMap = pointCloud(params.ptMap.Location(vIdx{:},:));
            subMap.Normal = params.ptMap.Normal(vIdx{:},:);
            subkdMap = createns(subMap.Location);
            params.CorresTF = XbkTF; % knn
            [Xk, Ck] = UpdataGene(subkdMap, subMap, ptHdl, Xbk, Cbk, XbkTF, params);
        end
        %
        Xk_1 = Xk;
        Ck_1 = Ck;
        %
        tmpTf = CPose2TF(Xk);
        VecTF = tmpTf * params.InvCalTF;
        vX(id, :) = Xk';
        vCov(:,:,id) = Ck;
        vehvX(id, :) = CTF2Pose(VecTF)';
        tNum(id, :) = ptHdl.Count;
        tTime(id, :) = toc;
        %
        Score = ScoreFun(params.kdMap, params.ptMap, ptOrg, tmpTf);
        tRatio(id,:) = [Score(1),Score(2)];
        %%
        GpsT = params.vGrdTF(1:3,end,id);
        EstT = VecTF(1:3,end);
        dErr = norm(GpsT-EstT);
        if Score(2) < 0.001 || dErr > 10
            btest = 1;
            fprintf('----------------The error of distance is %.2f, Score is %.2f---------------------\n', dErr,Score(1));
            break;
        end
        str0 = sprintf('C(%.4f), Dist(%s), Loss(%s), Frm(%03d/%03d), PtsNum(%04d/%06d), Ratio(%.4f,%.4f), dErr(%.2f), Time(%04dms)',params.ScaleC, DistType, params.RobustF ,id, length(params.SelRange),...
            ptHdl.Count, params.ptMap.Count, Score(1), Score(2), dErr, ceil(1000.0*tTime(id,:)));
        disp(str0);
        %% --------------------------------------------------------------
        params.isFirst = 2;
    end
    vFrmInfo.ptNum = params.ptNum;
    vFrmInfo.DataRoot = params.LocRoot;
    vFrmInfo.ExpName = params.ExpName;
    vFrmInfo.tTime = tTime;
    vFrmInfo.tRatio = tRatio;
    vFrmInfo.vX = vehvX;  % very important!!!!!!!!!
    vFrmInfo.vCov = vCov;
    vFrmInfo.tNum = tNum;
    DataSave = fullfile(pwd, params.SaveFolder);
    FileName = sprintf('%s/%s%s%s%sC%dN%d.mat',DataSave, TestName, params.MapType, params.DistType, params.RobustF, params.ScaleC, params.ptNum);
    save(FileName,'vFrmInfo');
end%
