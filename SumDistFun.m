function tDist = SumDistFun(GpsPose)
tic;
% tDist = 0;
% for i = 2 : 1 : length(GpsPose)
%     dGpsPose = GpsPose(i,1:3) - GpsPose(i-1,1:3);
%     dDist = sqrt(dGpsPose(1).^2 + dGpsPose(2).^2 + dGpsPose(3).^2 );
%     if dDist > 20       % 20m/s = 72km/h
%         dDist = 20;
%     end
%     tDist = tDist + dDist;
% end
% Second method
dPos = GpsPose(2:end,1:3) - GpsPose(1:end-1,1:3);
dDist = sqrt(dPos(:,1).^2 + dPos(:,2).^2 + dPos(:,3).^2 );
tDist = sum(dDist);
toc
end