function tDist = SumDistTFFun(vTF)
tic;
tDist = 0;
for i = 2 : 1 : size(vTF, 3)
    dt = vTF(1:3, end, i) - vTF(1:3, end, i-1);
    dDist = sqrt(dt(1).^2 + dt(2).^2 + dt(3).^2 );
    if dDist > 20       % 20m/s = 72km/h
        dDist = 20;
        break;
    end
    tDist = tDist + dDist;
end

toc
end