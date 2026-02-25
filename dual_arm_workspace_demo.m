function out = dual_arm_workspace_demo(varargin)
% DUAL_ARM_WORKSPACE_DEMO
% Simple, bootstrap-first dual-arm workspace demo for Doosan M1509.

opts = parseOptions(varargin{:});
rng(opts.Seed);

failStats = struct('ikFail',0,'collision',0,'reachFail',0,'attempts',0,'bootstrapMoves',0,'bootstrapScans',0);
out = struct('success',false,'scenario',struct(),'qL',[],'qR',[],'eeL',[],'eeR',[],'failStats',failStats);

if ~exist('importrobot','file') || ~license('test','Robotics_System_Toolbox')
    error('dual_arm_workspace_demo:MissingRST','Robotics System Toolbox 필요: importrobot/rigidBodyTree를 사용할 수 없습니다.');
end

urdfPath = fullfile(fileparts(mfilename('fullpath')),'m1509.urdf');
if ~isfile(urdfPath)
    error('dual_arm_workspace_demo:MissingURDF','m1509.urdf 파일을 찾을 수 없습니다: %s', urdfPath);
end

robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
qHome = homeConfiguration(robot);
qHome = reshape([qHome.JointPosition],1,[]);

bodyNames = robot.BodyNames;
eeName = bodyNames{end};
if opts.Verbose
    fprintf('[dual_arm_demo] EE frame: %s\n', eeName);
end

TL = trvec2tform(opts.BaseL);
TR = trvec2tform(opts.BaseR);

if isempty(opts.WorkspaceCenter)
    pL0 = worldPos(robot,qHome,eeName,TL);
    pR0 = worldPos(robot,qHome,eeName,TR);
    wsCenter = 0.5*(pL0 + pR0);
else
    wsCenter = opts.WorkspaceCenter(:).';
end

if strcmpi(opts.Mode,'bootstrap')
    [wsCenter, reachPair, failStats] = bootstrapCenter(robot,eeName,qHome,TL,TR,wsCenter,opts,failStats);
else
    reachPair = coarseReachability(robot,eeName,qHome,TL,TR,wsCenter,opts,18);
end

if min(reachPair) < 0.05
    failStats.reachFail = failStats.reachFail + 1;
end

[pAnchL,pAnchR] = generateAnchors(wsCenter,opts.WorkspaceSize,opts.Seed);
[tGrid,pL,pR] = makeDifferentTrajectories(pAnchL,pAnchR,opts.N);

[shoulderName, elbowName] = pickShoulderElbow(robot);
ikObj = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1e-3 1e-3 1e-3];

best = struct('ok',false,'score',inf,'qL',[],'qR',[],'eeL',[],'eeR',[]);
for attempt = 1:opts.MaxAttempts
    failStats.attempts = attempt;
    [qL,okL,eL,stL] = solveTrajIK(robot,ikObj,eeName,pL,TL,qHome,weights,shoulderName,elbowName,-1,opts.ElbowOutWeight);
    [qR,okR,eR,stR] = solveTrajIK(robot,ikObj,eeName,pR,TR,qHome,weights,shoulderName,elbowName,+1,opts.ElbowOutWeight);

    if ~(okL && okR)
        failStats.ikFail = failStats.ikFail + 1;
        if strcmpi(opts.Mode,'bootstrap')
            [pAnchL,pAnchR] = generateAnchors(wsCenter,opts.WorkspaceSize,opts.Seed+attempt);
            [~,pL,pR] = makeDifferentTrajectories(pAnchL,pAnchR,opts.N);
        end
        continue;
    end

    [isCol,colMin] = detectArmArmCollision(robot,qL,qR,TL,TR,opts.CollisionMargin,eeName);
    if isCol
        failStats.collision = failStats.collision + 1;
        [pAnchL,pAnchR] = generateAnchors(wsCenter,opts.WorkspaceSize,opts.Seed+attempt+17);
        [~,pL,pR] = makeDifferentTrajectories(pAnchL,pAnchR,opts.N);
        continue;
    end

    score = stL.meanPosErr + stR.meanPosErr + 0.05*colMin;
    if score < best.score
        best.ok = true; best.score = score;
        best.qL = qL; best.qR = qR;
        best.eeL = eL; best.eeR = eR;
    end

    if strcmpi(opts.Mode,'bootstrap')
        break;
    end
end

out.failStats = failStats;
out.scenario = struct('baseL',opts.BaseL,'baseR',opts.BaseR,'workspaceCenter',wsCenter,'workspaceSize',opts.WorkspaceSize,'reachability',reachPair,'eeFrame',eeName,'time',tGrid);

if ~best.ok
    fprintf('[dual_arm_demo] Seed=%d ws=[%.3f %.3f %.3f] baseL=[%.2f %.2f %.2f] baseR=[%.2f %.2f %.2f] reach=%.2f/%.2f success=0\n',...
        opts.Seed,opts.WorkspaceSize,opts.BaseL,opts.BaseR,reachPair(1),reachPair(2));
    fprintf('[dual_arm_demo] failStats: ikFail=%d collision=%d reachFail=%d attempts=%d\n',...
        failStats.ikFail,failStats.collision,failStats.reachFail,failStats.attempts);
    return;
end

out.success = true;
out.qL = best.qL;
out.qR = best.qR;
out.eeL = best.eeL;
out.eeR = best.eeR;

fprintf('[dual_arm_demo] Seed=%d ws=[%.3f %.3f %.3f] baseL=[%.2f %.2f %.2f] baseR=[%.2f %.2f %.2f] reach=%.2f/%.2f success=1\n',...
    opts.Seed,opts.WorkspaceSize,opts.BaseL,opts.BaseR,reachPair(1),reachPair(2));

if opts.Visualize
    visualizeDual(robot,best.qL,best.qR,best.eeL,best.eeR,TL,TR,wsCenter,opts.WorkspaceSize,eeName);
end
end

function opts = parseOptions(varargin)
p = inputParser;
addParameter(p,'Seed',1);
addParameter(p,'N',120);
addParameter(p,'WorkspaceSize',[0.45 0.30 0.30]);
addParameter(p,'WorkspaceCenter',[]);
addParameter(p,'BaseL',[-0.34 -0.12 0.00]);
addParameter(p,'BaseR',[ 0.34  0.12 0.00]);
addParameter(p,'Mode','bootstrap');
addParameter(p,'CollisionMargin',0.015);
addParameter(p,'ElbowOutWeight',0.2);
addParameter(p,'MaxAttempts',25);
addParameter(p,'Visualize',true);
addParameter(p,'Verbose',true);
parse(p,varargin{:});
opts = p.Results;
end

function [center, reachPair, failStats] = bootstrapCenter(robot,eeName,qHome,TL,TR,center0,opts,failStats)
reachPair = coarseReachability(robot,eeName,qHome,TL,TR,center0,opts,20);
center = center0;
if min(reachPair) >= 0.05
    return;
end

failStats.bootstrapMoves = failStats.bootstrapMoves + 1;
pL0 = worldPos(robot,qHome,eeName,TL);
pR0 = worldPos(robot,qHome,eeName,TR);
baseCenter = 0.5*(pL0 + pR0);

yOffsets = [-0.08 -0.04 0 0.04 0.08];
zOffsets = [-0.06 0 0.06];
bestScore = -inf;
bestCenter = baseCenter;
scanCount = 0;
for iy = 1:numel(yOffsets)
    for iz = 1:numel(zOffsets)
        if scanCount >= 12
            break;
        end
        c = baseCenter + [0 yOffsets(iy) zOffsets(iz)];
        r = coarseReachability(robot,eeName,qHome,TL,TR,c,opts,14);
        sc = min(r) + 0.2*sum(r);
        scanCount = scanCount + 1;
        if sc > bestScore
            bestScore = sc;
            bestCenter = c;
            reachPair = r;
        end
    end
end
failStats.bootstrapScans = failStats.bootstrapScans + scanCount;
center = bestCenter;
end

function rPair = coarseReachability(robot,eeName,qHome,TL,TR,center,opts,samples)
ik = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1e-3 1e-3 1e-3];
sz = opts.WorkspaceSize;
okL = 0; okR = 0;
for k = 1:samples
    u = (rand(1,3)-0.5).*sz;
    p = center + u;
    [~,infoL] = ik(eeName,trvec2tform(worldToLocal(TL,p)),weights,qHome);
    [~,infoR] = ik(eeName,trvec2tform(worldToLocal(TR,p)),weights,qHome);
    okL = okL + double(infoL.PoseErrorNorm < 5e-3);
    okR = okR + double(infoR.PoseErrorNorm < 5e-3);
end
rPair = [okL okR]/samples;
end

function [aL,aR] = generateAnchors(center,sz,seed)
rng(seed);
span = 0.42*sz;
aL = center + [-0.45*sz(1), -0.25*sz(2), 0] + [0 0.02 0.01];
aL = [aL; center + [-0.10*sz(1), -0.35*sz(2), 0.25*sz(3)]; center + [0.15*sz(1), -0.10*sz(2), -0.20*sz(3)]];
aR = center + [0.40*sz(1), 0.28*sz(2), 0.04*sz(3)];
aR = [aR; center + [0.08*sz(1), 0.40*sz(2), -0.25*sz(3)]; center + [-0.20*sz(1), 0.15*sz(2), 0.20*sz(3)]; center + [0.25*sz(1), 0.05*sz(2), -0.05*sz(3)]];
aL = aL + 0.03*(rand(size(aL))-0.5);
aR = aR + 0.03*(rand(size(aR))-0.5);
aL = clampToBox(aL,center,span);
aR = clampToBox(aR,center,span);
end

function pts = clampToBox(pts,center,span)
for i=1:3
    pts(:,i) = min(max(pts(:,i),center(i)-span(i)/2),center(i)+span(i)/2);
end
end

function [t,pL,pR] = makeDifferentTrajectories(aL,aR,N)
t = linspace(0,1,N).';
% Left: ease-in-out profile + 3 anchors
uL = 0.5 - 0.5*cos(pi*t);
tL = linspace(0,1,size(aL,1));
pL = [interp1(tL,aL(:,1),uL,'pchip'), interp1(tL,aL(:,2),uL,'pchip'), interp1(tL,aL(:,3),uL,'pchip')];
% Right: trapezoid-like time warp + phase offset + 4 anchors
phase = mod(t + 0.18,1.0);
uR = trapProfile(phase,0.18,0.72);
tR = linspace(0,1,size(aR,1));
pR = [interp1(tR,aR(:,1),uR,'pchip'), interp1(tR,aR(:,2),uR,'pchip'), interp1(tR,aR(:,3),uR,'pchip')];
end

function u = trapProfile(t,t1,t2)
u = zeros(size(t));
for i=1:numel(t)
    x = t(i);
    if x < t1
        u(i) = 0.5*(x/t1)^2;
    elseif x < t2
        u(i) = 0.5 + 0.45*(x-t1)/(t2-t1);
    else
        y = (x-t2)/(1-t2);
        u(i) = 0.95 + 0.05*(1 - (1-y)^2);
    end
end
u = min(max(u,0),1);
end

function [shoulder, elbow] = pickShoulderElbow(robot)
n = numel(robot.BodyNames);
shoulder = robot.BodyNames{min(2,n)};
elbow = robot.BodyNames{min(4,n)};
end

function [qTraj,ok,eeTraj,stats] = solveTrajIK(robot,ikObj,eeName,pWorld,Tbase,qInit,weights,shoulder,elbow,sideSign,elbowW)
N = size(pWorld,1);
qTraj = zeros(N,numel(qInit));
eeTraj = zeros(N,3);
qPrev = qInit;
ikOk = true;
err = zeros(N,1);
for k = 1:N
    pLocal = worldToLocal(Tbase,pWorld(k,:));
    Tgoal = trvec2tform(pLocal);
    [qBest,eBest] = ikMultiStart(robot,ikObj,eeName,Tgoal,weights,qPrev,qInit,shoulder,elbow,sideSign,elbowW,Tbase);
    qTraj(k,:) = qBest;
    qPrev = qBest;
    err(k) = eBest;
    if eBest > 2e-2
        ikOk = false;
    end
    eeTraj(k,:) = worldPos(robot,qBest,eeName,Tbase);
end
ok = ikOk;
stats = struct('meanPosErr',mean(err),'maxPosErr',max(err));
end

function [qBest,eBest] = ikMultiStart(robot,ikObj,eeName,Tgoal,weights,qPrev,qHome,shoulder,elbow,sideSign,elbowW,Tbase)
seeds = [qPrev; qHome; qHome + 0.15*randn(1,numel(qHome))];
qBest = qPrev;
eBest = inf;
for i = 1:size(seeds,1)
    [qCand,info] = ikObj(eeName,Tgoal,weights,seeds(i,:));
    pErr = info.PoseErrorNorm;
    eScore = pErr + elbowW * elbowOutPenalty(robot,qCand,shoulder,elbow,sideSign,Tbase);
    if pErr > 4e-2
        [qCand2,info2] = ikObj(eeName,Tgoal,[1 1 1 1e-4 1e-4 1e-4],qCand);
        pErr2 = info2.PoseErrorNorm;
        eScore2 = pErr2 + elbowW * elbowOutPenalty(robot,qCand2,shoulder,elbow,sideSign,Tbase);
        if eScore2 < eScore
            qCand = qCand2;
            pErr = pErr2;
            eScore = eScore2;
        end
    end
    if eScore < eBest
        eBest = pErr;
        qBest = qCand;
    end
end
end

function pen = elbowOutPenalty(robot,q,shoulder,elbow,sideSign,Tbase)
pS = worldBodyPos(robot,q,shoulder,Tbase);
pE = worldBodyPos(robot,q,elbow,Tbase);
local = pE - pS;
want = sideSign * local(2);
pen = max(0,-want);
end

function [isCol,minDist] = detectArmArmCollision(robot,qL,qR,TL,TR,margin,eeName)
N = size(qL,1);
bodyNames = robot.BodyNames;
idx = 1:min(numel(bodyNames),6);
radius = linspace(0.07,0.045,numel(idx));
minDist = inf;
isCol = false;
for k = 1:N
    pEeL = worldPos(robot,qL(k,:),eeName,TL);
    pEeR = worldPos(robot,qR(k,:),eeName,TR);
    if norm(pEeL - pEeR) > 0.85
        continue;
    end
    ptsL = chainPoints(robot,qL(k,:),TL,bodyNames(idx));
    ptsR = chainPoints(robot,qR(k,:),TR,bodyNames(idx));
    for i = 1:size(ptsL,1)-1
        for j = 1:size(ptsR,1)-1
            d = segmentSegmentDistance(ptsL(i,:),ptsL(i+1,:),ptsR(j,:),ptsR(j+1,:));
            th = radius(i) + radius(j) + margin;
            minDist = min(minDist,d-th);
            if d < th
                isCol = true;
                return;
            end
        end
    end
end
if isinf(minDist), minDist = 1.0; end
end

function pts = chainPoints(robot,q,Tbase,names)
pts = zeros(numel(names)+1,3);
pts(1,:) = Tbase(1:3,4).';
for i = 1:numel(names)
    pts(i+1,:) = worldBodyPos(robot,q,names{i},Tbase);
end
end

function d = segmentSegmentDistance(p1,q1,p2,q2)
u = q1 - p1;
v = q2 - p2;
w = p1 - p2;
a = dot(u,u); b = dot(u,v); c = dot(v,v); d1 = dot(u,w); e = dot(v,w);
D = a*c - b*b;
sc = 0; sN = 0; sD = D;
tc = 0; tN = 0; tD = D;
if D < 1e-12
    sN = 0; sD = 1; tN = e; tD = c;
else
    sN = (b*e - c*d1);
    tN = (a*e - b*d1);
    if sN < 0
        sN = 0; tN = e; tD = c;
    elseif sN > sD
        sN = sD; tN = e + b; tD = c;
    end
end
if tN < 0
    tN = 0;
    if -d1 < 0
        sN = 0;
    elseif -d1 > a
        sN = sD;
    else
        sN = -d1; sD = a;
    end
elseif tN > tD
    tN = tD;
    if (-d1 + b) < 0
        sN = 0;
    elseif (-d1 + b) > a
        sN = sD;
    else
        sN = (-d1 + b); sD = a;
    end
end
if abs(sN) < 1e-12, sc = 0; else, sc = sN / sD; end
if abs(tN) < 1e-12, tc = 0; else, tc = tN / tD; end
dP = w + sc*u - tc*v;
d = norm(dP);
end

function p = worldPos(robot,q,body,Tbase)
T = Tbase * getTransform(robot,q,body);
p = T(1:3,4).';
end

function p = worldBodyPos(robot,q,body,Tbase)
T = Tbase * getTransform(robot,q,body);
p = T(1:3,4).';
end

function pLocal = worldToLocal(Tbase,pWorld)
p = [pWorld(:);1];
pl = Tbase\p;
pLocal = pl(1:3).';
end

function visualizeDual(robot,qL,qR,eeL,eeR,TL,TR,center,sz,eeName)
figure('Name','Dual Arm Workspace Demo');
ax = axes; hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel('X'); ylabel('Y'); zlabel('Z');
plot3(eeL(:,1),eeL(:,2),eeL(:,3),'b-','LineWidth',1.5);
plot3(eeR(:,1),eeR(:,2),eeR(:,3),'r-','LineWidth',1.5);
drawBox(center,sz);

for k = 1:3:size(qL,1)
    cla(ax);
    drawBox(center,sz);
    plot3(eeL(:,1),eeL(:,2),eeL(:,3),'b-','LineWidth',1.2);
    plot3(eeR(:,1),eeR(:,2),eeR(:,3),'r-','LineWidth',1.2);
    show(robot,qL(k,:),'Parent',ax,'PreservePlot',true,'Frames','off');
    h = findobj(ax,'Type','Patch');
    for i = 1:numel(h), h(i).FaceColor = [0.2 0.4 1.0]; h(i).FaceAlpha = 0.25; end
    show(robot,qR(k,:),'Parent',ax,'PreservePlot',true,'Frames','off');
    h2 = findobj(ax,'Type','Patch');
    for i = 1:numel(h2), if h2(i).FaceAlpha<0.5, h2(i).FaceColor = [1.0 0.2 0.2]; end, end
    title(sprintf('Dual arm demo (%s), step %d/%d',eeName,k,size(qL,1)));
    drawnow;
end
end

function drawBox(c,s)
[X,Y,Z] = ndgrid([-0.5 0.5]*s(1),[-0.5 0.5]*s(2),[-0.5 0.5]*s(3));
V = [X(:)+c(1),Y(:)+c(2),Z(:)+c(3)];
F = [1 2 4 3;5 6 8 7;1 2 6 5;3 4 8 7;1 3 7 5;2 4 8 6];
patch('Vertices',V,'Faces',F,'FaceColor',[0.6 0.8 0.6],'FaceAlpha',0.05,'EdgeColor',[0 0.6 0]);
end
