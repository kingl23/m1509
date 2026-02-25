function dual_target_workspace(opts)
% DUAL_TARGET_WORKSPACE
% Bootstrap-first dual-arm demo:
%   - default mode='bootstrap' : prioritize finding at least 1 valid trajectory fast
%   - optional mode='strict'   : stronger elbow-out/collision constraints
%
% Core goals:
%   1) Seed=1 default run should find at least one trajectory and visualize.
%   2) Avoid early death from heavy scenario filtering.
%   3) Print immediate diagnostics for EE frame / IK residual / reachability causes.

if nargin < 1
    opts = struct();
end

%% ------------------------- user options -------------------------
seed = getOpt(opts, "Seed", 1);
rng(seed);

mode = string(getOpt(opts, "Mode", "bootstrap")); % 'bootstrap' | 'strict'
verbose = getOpt(opts, "Verbose", true);
verboseCollision = getOpt(opts, "VerboseCollision", true);

Ttotal = getOpt(opts, "Ttotal", 6.0);
fps = getOpt(opts, "Fps", 20);
N = max(50, round(Ttotal*fps));

% Workspace size MUST stay fixed by requirement.
ellABC = [0.450, 0.300, 0.300];

% Scenario search now optional (default OFF as requested)
useScenarioSearch = getOpt(opts, "UseScenarioSearch", false);

% Base defaults: wide to reduce inter-arm conflict and improve right-arm reach.
baseLeftXYZ  = getOpt(opts, "BaseLeftXYZ",  [-0.34 -0.12 0.00]);
baseRightXYZ = getOpt(opts, "BaseRightXYZ", [+0.34 +0.12 0.00]);
baseYawDegL  = getOpt(opts, "BaseYawDegL", 0);
baseYawDegR  = getOpt(opts, "BaseYawDegR", 0);

forwardOffset = getOpt(opts, "ForwardOffset", 0.50);
backMargin    = getOpt(opts, "BackMargin", 0.05);
zRangeHalf    = getOpt(opts, "ZRangeHalf", 0.12);

% If user provides center, we use it. otherwise auto-estimate from neutral FK.
userCenter = getOpt(opts, "WorkspaceCenter", []);

% IK / attempt budgets (light)
maxTrajAttempts = getOpt(opts, "MaxTrajAttempts", 35);
coarseScanCenters = getOpt(opts, "CoarseScanCenters", 15); % fallback center scan count
coarseSamplesPerArm = getOpt(opts, "CoarseSamplesPerArm", 10);
anchorNeeded = getOpt(opts, "AnchorNeeded", 3);

xKeep = getOpt(opts, "XKeep", 0.09);
minEEDist = getOpt(opts, "MinEEDist", 0.18);

if mode == "strict"
    elbowOutSoftW = 1.6;
    elbowOutHard = 0.010;
    collisionMargin = 0.016;
    broadMargin = 0.09;
else
    elbowOutSoftW = 0.45;
    elbowOutHard = -0.020;  % effectively soft in bootstrap
    collisionMargin = 0.000; % loose first-stage gate
    broadMargin = 0.12;
end

numEvalTraj = getOpt(opts, "NumEvalTraj", 0);

%% -------------------------- path / load --------------------------
baseDir = fileparts(mfilename("fullpath"));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

[robotRaw, eeRaw, qHome] = loadRobotAutoEE(baseDir);

[robotL, robotR] = placeDualRobots(robotRaw, baseLeftXYZ, baseRightXYZ, baseYawDegL, baseYawDegR);
eeL = resolveEEName(robotL, eeRaw);
eeR = resolveEEName(robotR, eeRaw);

[shL, elL] = pickShoulderElbowBodies(robotL);
[shR, elR] = pickShoulderElbowBodies(robotR);

ikL = makeIKBackend(robotL);
ikR = makeIKBackend(robotR);

armBodiesL = pickArmBodies(robotL);
armBodiesR = pickArmBodies(robotR);
radMapL = buildArmCapsuleRadiusMap(armBodiesL);
radMapR = buildArmCapsuleRadiusMap(armBodiesR);

if verbose
    fprintf("\n[dual_target_workspace] Seed=%d mode=%s N=%d\n", seed, mode, N);
    fprintf("  IK backend L=%s R=%s\n", ikL.Mode, ikR.Mode);
    fprintf("  Base L=(%.3f %.3f %.3f) R=(%.3f %.3f %.3f)\n", baseLeftXYZ, baseRightXYZ);
    fprintf("  Base yaw deg L/R=(%.1f, %.1f) | forwardOffset=%.3f\n", baseYawDegL, baseYawDegR, forwardOffset);
end

%% -------------------- EE frame quick diagnostics --------------------
TL0 = getTransform(robotL, qHome, eeL);
TR0 = getTransform(robotR, qHome, eeR);
if verbose
    fprintf("  EE(L)='%s' neutral pos=(%.3f %.3f %.3f) zAxis=(%.2f %.2f %.2f)\n", eeL, TL0(1:3,4), TL0(1:3,3));
    fprintf("  EE(R)='%s' neutral pos=(%.3f %.3f %.3f) zAxis=(%.2f %.2f %.2f)\n", eeR, TR0(1:3,4), TR0(1:3,3));
end

%% -------------------- center selection (bootstrap) --------------------
minCenterY = max(baseLeftXYZ(2), baseRightXYZ(2)) + forwardOffset;
if isempty(userCenter)
    centerAuto = 0.5*(TL0(1:3,4).' + TR0(1:3,4).') + [0.00, 0.05, 0.00];
else
    centerAuto = userCenter;
end
centerAuto(1) = 0.0; % keep workspace centered in x
if centerAuto(2) < minCenterY
    if verbose && ~isempty(userCenter)
        fprintf("  userCenter adjusted to satisfy forward constraint: y %.3f -> %.3f\n", centerAuto(2), minCenterY);
    end
    centerAuto(2) = minCenterY;
end

% Optional old-style scenario search can be enabled, but default is OFF.
if useScenarioSearch
    [centerSel, scenarioLog] = optionalScenarioCenterSearch(robotL,robotR,ikL,ikR,eeL,eeR,qHome,centerAuto,ellABC,xKeep,coarseSamplesPerArm,coarseScanCenters,verbose,minCenterY,backMargin,zRangeHalf);
else
    [centerSel, scenarioLog] = fallbackCenterScan(robotL,robotR,ikL,ikR,eeL,eeR,qHome,centerAuto,ellABC,xKeep,coarseSamplesPerArm,coarseScanCenters,verbose,minCenterY,backMargin,zRangeHalf);
end
centerSel(1) = 0.0;
if centerSel(2) < minCenterY
    centerSel(2) = minCenterY;
end

frontOK = (centerSel(2) >= minCenterY);
if verbose
    fprintf("  Workspace center=(%.3f %.3f %.3f), score=%.3f\n", centerSel, scenarioLog.BestScore);
    fprintf("  workspace is in front of both bases: %d (minY=%.3f)\n", frontOK, minCenterY);
end

%% -------------------- quick IK residual probe --------------------
pProbeL = centerSel + [-0.12, -0.02, 0.00];
pProbeR = centerSel + [+0.12, +0.02, 0.00];
[qProbeL, okProbeL, infoProbeL] = solveIKPoint(robotL, ikL, eeL, pProbeL, qHome, qHome, -1, shL, elL, elbowOutSoftW);
[qProbeR, okProbeR, infoProbeR] = solveIKPoint(robotR, ikR, eeR, pProbeR, qHome, qHome, +1, shR, elR, elbowOutSoftW);
if verbose
    fprintf("  Probe IK: L ok=%d residual=%.4f | R ok=%d residual=%.4f\n", okProbeL, infoProbeL.PosErr, okProbeR, infoProbeR.PosErr);
end

%% -------------------- reachable anchors --------------------
reachOpt = struct("Samples", 60, "Needed", anchorNeeded, "XKeep", xKeep, "ElbowOutSoftW", elbowOutSoftW);
[aL, qAL, rStatL] = collectReachableAnchors(robotL, ikL, eeL, qHome, centerSel, ellABC, -1, shL, elL, reachOpt, backMargin, zRangeHalf);
[aR, qAR, rStatR] = collectReachableAnchors(robotR, ikR, eeR, qHome, centerSel, ellABC, +1, shR, elR, reachOpt, backMargin, zRangeHalf);

if verbose
    fprintf("  Reachability: L %d/%d (mean residual=%.4f), R %d/%d (mean residual=%.4f)\n", ...
        rStatL.Success, rStatL.Tested, rStatL.MeanResidual, rStatR.Success, rStatR.Tested, rStatR.MeanResidual);
end

if numel(aL) < anchorNeeded || numel(aR) < anchorNeeded
    error("Reachability too low after fallback scan. Cause: center/base likely unreachable or IK solver not converging.");
end

%% -------------------- build & solve trajectory --------------------
Q1 = []; Q2 = []; P1 = []; P2 = []; meta = struct();
finalDiag = struct();

for attempt = 1:maxTrajAttempts
    [P1c, P2c, metaC] = generateDualTrajectory(aL, aR, N, centerSel, ellABC, xKeep, minEEDist, mode, attempt, backMargin, zRangeHalf);

    if any(vecnorm(P1c-P2c,2,2) < minEEDist)
        continue;
    end

    [Q1c, stL] = solveTrajectorySequence(robotL, ikL, eeL, P1c, qHome, qAL, -1, shL, elL, elbowOutSoftW);
    [Q2c, stR] = solveTrajectorySequence(robotR, ikR, eeR, P2c, qHome, qAR, +1, shR, elR, elbowOutSoftW);

    if stL.SuccessRate < 0.80 || stR.SuccessRate < 0.80
        if verbose && mod(attempt,8)==0
            fprintf("  [attempt %d] IK low: L=%.1f%% R=%.1f%%\n", attempt, 100*stL.SuccessRate, 100*stR.SuccessRate);
        end
        continue;
    end

    [isCol, cInfo] = trajectoryArmCollision(robotL,Q1c,armBodiesL,radMapL,robotR,Q2c,armBodiesR,radMapR,collisionMargin,broadMargin);
    if isCol
        if verboseCollision
            fprintf("  [attempt %d] ArmArm collision t=%d pair=(%s <-> %s) d=%.4f th=%.4f\n", ...
                attempt, cInfo.t, cInfo.linkL, cInfo.linkR, cInfo.d, cInfo.th);
        end
        continue;
    end

    mL = elbowMetrics(robotL,Q1c,shL,elL,-1);
    mR = elbowMetrics(robotR,Q2c,shR,elR,+1);

    if mode == "strict"
        if mL.LocalMean < elbowOutHard || mR.LocalMean < elbowOutHard
            continue;
        end
    end

    Q1 = Q1c; Q2 = Q2c; P1 = P1c; P2 = P2c; meta = metaC;
    finalDiag.IKLeft = stL; finalDiag.IKRight = stR;
    finalDiag.ElbowLeft = mL; finalDiag.ElbowRight = mR;
    break;
end

if isempty(Q1)
    error("No trajectory accepted. Root-cause hint: likely IK convergence instability or unreachable center for one arm.");
end

if verbose
    fprintf("\n[success] IK L/R = %.1f%% / %.1f%% | elbow keep L/R = %.1f%% / %.1f%%\n", ...
        100*finalDiag.IKLeft.SuccessRate, 100*finalDiag.IKRight.SuccessRate, ...
        100*finalDiag.ElbowLeft.KeepRatio, 100*finalDiag.ElbowRight.KeepRatio);
end

if numEvalTraj > 0
    ev = batchEval(robotL,robotR,ikL,ikR,eeL,eeR,qHome,aL,aR,qAL,qAR,shL,elL,shR,elR,centerSel,ellABC,mode,numEvalTraj,xKeep,minEEDist,armBodiesL,armBodiesR,radMapL,radMapR,collisionMargin,broadMargin,elbowOutSoftW,backMargin,zRangeHalf);
    fprintf("[batch] IK mean L/R=%.1f%% / %.1f%% | elbow keep L/R=%.1f%% / %.1f%% | arm-arm pass=%.1f%%\n", ...
        100*ev.IKRateL,100*ev.IKRateR,100*ev.ElbowKeepL,100*ev.ElbowKeepR,100*ev.CollisionPass);
end

visualizeDual(robotL,robotR,Q1,Q2,P1,P2,centerSel,ellABC,meta,fps);

end

%% =====================================================================
%%                               HELPERS
%% =====================================================================

function [centerSel, logOut] = optionalScenarioCenterSearch(robotL,robotR,ikL,ikR,eeL,eeR,qHome,center0,ellABC,xKeep,coarseSamples,scanN,verbose,minCenterY,backMargin,zRangeHalf)
% Kept for compatibility. Now delegates to lightweight fallback scan.
[centerSel, logOut] = fallbackCenterScan(robotL,robotR,ikL,ikR,eeL,eeR,qHome,center0,ellABC,xKeep,coarseSamples,scanN,verbose,minCenterY,backMargin,zRangeHalf);
end

function [centerBest, logOut] = fallbackCenterScan(robotL,robotR,ikL,ikR,eeL,eeR,qHome,center0,ellABC,xKeep,coarseSamples,scanN,verbose,minCenterY,backMargin,zRangeHalf)
% Data-driven center scan:
% 1) Build candidate centers around neutral-FK-based center.
% 2) Coarse IK success test (10-ish samples per arm) for each center.
% 3) Choose best center with highest bilateral reach score.

center0(1) = 0.0;
if center0(2) < minCenterY
    center0(2) = minCenterY;
end
cand = generateCenterCandidates(center0, scanN, minCenterY);
score = -inf(size(cand,1),1);

bestMeanResidual = inf;
for i = 1:size(cand,1)
    c = cand(i,:);
    [sL, rL] = coarseReachScore(robotL,ikL,eeL,qHome,c,ellABC,-1,xKeep,coarseSamples,backMargin,zRangeHalf);
    [sR, rR] = coarseReachScore(robotR,ikR,eeR,qHome,c,ellABC,+1,xKeep,coarseSamples,backMargin,zRangeHalf);

    % bilateral score: both arms must be good.
    score(i) = min(sL,sR) + 0.25*(sL+sR);
    meanRes = 0.5*(rL+rR);

    if score(i) > max(score(1:i))
        bestMeanResidual = meanRes;
    end

    if verbose
        fprintf("  [center scan %02d] c=(%.3f %.3f %.3f) reachL=%.2f reachR=%.2f meanRes=%.4f\n", i, c, sL, sR, meanRes);
    end
end

[~, idx] = max(score);
centerBest = cand(idx,:);
centerBest(1)=0.0;
if centerBest(2)<minCenterY, centerBest(2)=minCenterY; end

logOut = struct();
logOut.BestScore = score(idx);
logOut.BestMeanResidual = bestMeanResidual;
end

function cand = generateCenterCandidates(c0, n, minCenterY)
% lightweight y/z sweep around c0
if n < 4
    n = 4;
end

cand = zeros(n,3);
cand(1,:) = [0.0, max(c0(2),minCenterY), c0(3)];

k = 2;
for dy = [-0.10 -0.06 -0.03 0 0.03 0.06 0.10]
    for dz = [-0.08 -0.04 0 0.04 0.08]
        if k > n
            return;
        end
        yc = max(c0(2)+dy, minCenterY);
        cand(k,:) = [0.0, yc, c0(3)+dz];
        k = k + 1;
    end
end

while k <= n
    yc = max(c0(2)+0.02*randn(), minCenterY);
    cand(k,:) = [0.0, yc, c0(3)+0.02*randn()];
    k = k + 1;
end
end

function [ratio, meanResidual] = coarseReachScore(robot,ik,eeName,qHome,center,abc,sideSign,xKeep,samples,backMargin,zRangeHalf)
okN = 0;
res = nan(samples,1);
qPrev = qHome;
for i = 1:samples
    p = samplePointInEllipsoid(center, abc, 0.65);
    p(1) = sideSign*max(abs(p(1)), xKeep);
    p(2) = max(p(2), center(2)-backMargin);
    p(3) = min(max(p(3), center(3)-zRangeHalf), center(3)+zRangeHalf);
    [q, ok, info] = solveIKPoint(robot,ik,eeName,p,qPrev,qHome,sideSign,'','',0.0);
    if ok
        okN = okN + 1;
        qPrev = q;
        res(i) = info.PosErr;
    end
end
ratio = okN/max(1,samples);
meanResidual = mean(res(~isnan(res)));
if ~isfinite(meanResidual)
    meanResidual = 1e3;
end
end

function [anchors, anchorQ, stat] = collectReachableAnchors(robot,ik,eeName,qHome,center,abc,sideSign,shBody,elBody,opt,backMargin,zRangeHalf)
samples = getOpt(opt,"Samples",60);
needed = getOpt(opt,"Needed",3);
xKeep = getOpt(opt,"XKeep",0.09);
elbowW = getOpt(opt,"ElbowOutSoftW",0.3);

anchors = {};
anchorQ = {};
qPrev = qHome;
residuals = [];

for i = 1:samples
    p = samplePointInEllipsoid(center, abc, 0.80);
    p(1) = sideSign*max(abs(p(1)), xKeep);
    p(2) = max(p(2), center(2)-backMargin);
    p(3) = min(max(p(3), center(3)-zRangeHalf), center(3)+zRangeHalf);

    seedAlt = perturbSeed(qPrev, 0.16);
    [q1, ok1, info1] = solveIKPoint(robot,ik,eeName,p,qPrev,qHome,sideSign,shBody,elBody,elbowW);
    [q2, ok2, info2] = solveIKPoint(robot,ik,eeName,p,seedAlt,qHome,sideSign,shBody,elBody,elbowW);

    q = q1; ok = ok1; info = info1;
    if ok2 && (~ok1 || info2.Score < info1.Score)
        q = q2; ok = ok2; info = info2;
    end

    if ~ok || info.PosErr > 0.05
        continue;
    end

    anchors{end+1} = p; %#ok<AGROW>
    anchorQ{end+1} = q; %#ok<AGROW>
    residuals(end+1) = info.PosErr; %#ok<AGROW>
    qPrev = q;

    if numel(anchors) >= needed
        break;
    end
end

stat = struct();
stat.Tested = samples;
stat.Success = numel(anchors);
if isempty(residuals)
    stat.MeanResidual = inf;
else
    stat.MeanResidual = mean(residuals);
end
end

function [q, ok, info] = solveIKPoint(robot,ik,eeName,pGoal,qSeed,qRef,sideSign,shBody,elBody,elbowOutSoftW)
% Success-first IK point solver:
% - position-priority IK
% - mild soft elbow-out penalty (can be zero)

Tgoal = [eye(3), pGoal(:); 0 0 0 1];
seedPool = {qSeed, qRef, perturbSeed(qSeed,0.20)};

bestScore = inf;
bestQ = qSeed;
bestErr = inf;

for i = 1:numel(seedPool)
    q0 = seedPool{i};
    [qCand, solved] = solveSingleIK(robot,ik,eeName,Tgoal,q0);
    if ~solved || any(~isfinite(qCand))
        continue;
    end

    Tcur = getTransform(robot, qCand, eeName);
    posErr = norm(Tcur(1:3,4).' - pGoal, 2);

    elbowPenalty = 0;
    if ~isempty(shBody) && ~isempty(elBody)
        [elLocal, elCenter] = elbowOutScalars(robot,qCand,shBody,elBody,sideSign);
        elbowPenalty = softplus(0.006-elLocal,35) + 0.8*softplus(0.008-elCenter,30);
    end

    score = 18*posErr + elbowOutSoftW*elbowPenalty + 0.03*norm(qCand-qRef)^2;

    if score < bestScore
        bestScore = score;
        bestQ = qCand;
        bestErr = posErr;
    end
end

q = bestQ;
ok = isfinite(bestErr) && bestErr < 0.06;
info = struct('PosErr',bestErr,'Score',bestScore);
end

function [Q, stat] = solveTrajectorySequence(robot,ik,eeName,P,qHome,anchorQ,sideSign,shBody,elBody,elbowOutSoftW)
N = size(P,1);
Q = zeros(N, numel(qHome));
succ = false(N,1);
res = nan(N,1);

qPrev = qHome;
for k = 1:N
    idx = 1 + mod(k-1, numel(anchorQ));
    qA = anchorQ{idx};

    [q, ok, info] = solveIKPoint(robot,ik,eeName,P(k,:),qPrev,qA,sideSign,shBody,elBody,elbowOutSoftW);

    if ok
        Q(k,:) = q;
        qPrev = q;
        succ(k) = true;
        res(k) = info.PosErr;
    else
        if k == 1
            Q(k,:) = qHome;
        else
            Q(k,:) = Q(k-1,:);
        end
        qPrev = Q(k,:);
    end
end

% one lightweight repair pass on failures
bad = find(~succ);
for i = 1:numel(bad)
    k = bad(i);
    qN = neighborSeed(Q,k,qHome);
    [q, ok, info] = solveIKPoint(robot,ik,eeName,P(k,:),qN,qHome,sideSign,shBody,elBody,elbowOutSoftW);
    if ok
        Q(k,:) = q;
        succ(k) = true;
        res(k) = info.PosErr;
    end
end

stat = struct();
stat.SuccessRate = mean(succ);
stat.SuccessMask = succ;
stat.MeanResidual = mean(res(succ));
if ~isfinite(stat.MeanResidual)
    stat.MeanResidual = inf;
end
end

function [P1,P2,meta] = generateDualTrajectory(anchorL,anchorR,N,center,abc,xKeep,minEEDist,mode,attempt,backMargin,zRangeHalf)
A1 = cell2mat(anchorL(:));
A2 = cell2mat(anchorR(:));
A1 = [A1; A1(1,:)];
A2 = [A2; A2(1,:)];

if mod(attempt,2)==0
    A2 = circshift(A2,1,1);
end

t = linspace(0,1,N).';
t1 = linspace(0,1,size(A1,1));
t2 = linspace(0,1,size(A2,1));

if mode == "strict"
    vL = 0.88 + 0.18*rand();
    vR = 1.05 + 0.35*rand();
    ph = 0.20 + 0.20*rand();
    uL = mod(vL*t,1.0);
    uR = mod(vR*t+ph,1.0);
    tauL = 0.5 - 0.5*cos(pi*uL);
    tauR = trapezoidWarp(uR,0.16,0.72);
else
    % bootstrap: still different, but simpler and easier to solve
    vL = 0.95;
    vR = 1.10;
    ph = 0.16;
    uL = mod(vL*t,1.0);
    uR = mod(vR*t+ph,1.0);
    tauL = uL;
    tauR = 0.5 - 0.5*cos(pi*uR);
end

P1 = [interp1(t1,A1(:,1),tauL,'pchip'), interp1(t1,A1(:,2),tauL,'pchip'), interp1(t1,A1(:,3),tauL,'pchip')];
P2 = [interp1(t2,A2(:,1),tauR,'pchip'), interp1(t2,A2(:,2),tauR,'pchip'), interp1(t2,A2(:,3),tauR,'pchip')];

[P1,P2] = clampInsideEllipsoid(P1,P2,center,abc,0.95);
P1(:,1) = min(P1(:,1), -xKeep);
P2(:,1) = max(P2(:,1), +xKeep);
P1(:,2) = max(P1(:,2), center(2)-backMargin);
P2(:,2) = max(P2(:,2), center(2)-backMargin);
P1(:,3) = min(max(P1(:,3), center(3)-zRangeHalf), center(3)+zRangeHalf);
P2(:,3) = min(max(P2(:,3), center(3)-zRangeHalf), center(3)+zRangeHalf);

% If still too close, shift y slightly apart
closeIdx = vecnorm(P1-P2,2,2) < minEEDist;
if any(closeIdx)
    P1(closeIdx,2) = P1(closeIdx,2) - 0.03;
    P2(closeIdx,2) = P2(closeIdx,2) + 0.03;
end

meta = struct('LeftSpeed',vL,'RightSpeed',vR,'Phase',ph,'Mode',mode);
end

function tau = trapezoidWarp(u,aFrac,dStart)
u = max(0,min(1,u));
tau = zeros(size(u));
i1 = (u < aFrac);
i2 = (u >= aFrac) & (u <= dStart);
i3 = (u > dStart);
if any(i1)
    s = u(i1)/max(aFrac,1e-6);
    tau(i1) = 0.5*aFrac*s.^2;
end
if any(i2)
    tau(i2) = 0.5*aFrac + (u(i2)-aFrac);
end
if any(i3)
    d = max(1-dStart,1e-6);
    s = (u(i3)-dStart)/d;
    y0 = 0.5*aFrac + (dStart-aFrac);
    tau(i3) = y0 + d*(s - 0.5*s.^2);
end
tau = tau - min(tau);
tau = tau / max(max(tau),1e-9);
end

function [isCol, info] = trajectoryArmCollision(robotL,QL,armBodiesL,radL,robotR,QR,armBodiesR,radR,margin,broadMargin)
isCol = false;
info = struct('t',-1,'linkL','','linkR','','d',nan,'th',nan);
N = size(QL,1);
for k = 1:N
    [col, c] = armArmCapsuleCollision(robotL,QL(k,:),armBodiesL,radL,robotR,QR(k,:),armBodiesR,radR,margin,broadMargin);
    if col
        isCol = true;
        info.t = k;
        info.linkL = c.Link1;
        info.linkR = c.Link2;
        info.d = c.Distance;
        info.th = c.Threshold;
        return;
    end
end
end

function m = elbowMetrics(robot,Q,shoulderBody,elbowBody,sideSign)
N = size(Q,1);
outLocal = zeros(N,1);
outCenter = zeros(N,1);
for k = 1:N
    [outLocal(k), outCenter(k)] = elbowOutScalars(robot,Q(k,:),shoulderBody,elbowBody,sideSign);
end
keep = (outLocal > 0) & (outCenter > 0);
m = struct('KeepRatio',mean(keep),'LocalMean',mean(outLocal),'CenterMean',mean(outCenter));
end

function [outLocal,outCenter] = elbowOutScalars(robot,q,shoulderBody,elbowBody,sideSign)
if isempty(shoulderBody) || isempty(elbowBody)
    outLocal = 0; outCenter = 0; return;
end
Tsh = getTransform(robot,q,shoulderBody);
Tel = getTransform(robot,q,elbowBody);
d = Tel(1:3,4)-Tsh(1:3,4);
outLocal = sideSign*d(1);
outCenter = sideSign*Tel(1,4);
end

function ev = batchEval(robotL,robotR,ikL,ikR,eeL,eeR,qHome,aL,aR,qAL,qAR,shL,elL,shR,elR,center,abc,mode,nEval,xKeep,minEEDist,armBodiesL,armBodiesR,radL,radR,margin,broadMargin,elbowOutSoftW,backMargin,zRangeHalf)
ikLsum = 0; ikRsum = 0; elLsum = 0; elRsum = 0; passCol=0;
for i = 1:nEval
    [P1,P2] = generateDualTrajectory(aL,aR,max(50,round(5*20)),center,abc,xKeep,minEEDist,mode,i,backMargin,zRangeHalf);
    [Q1,stL] = solveTrajectorySequence(robotL,ikL,eeL,P1,qHome,qAL,-1,shL,elL,elbowOutSoftW);
    [Q2,stR] = solveTrajectorySequence(robotR,ikR,eeR,P2,qHome,qAR,+1,shR,elR,elbowOutSoftW);
    mL = elbowMetrics(robotL,Q1,shL,elL,-1);
    mR = elbowMetrics(robotR,Q2,shR,elR,+1);
    [col,~] = trajectoryArmCollision(robotL,Q1,armBodiesL,radL,robotR,Q2,armBodiesR,radR,margin,broadMargin);
    ikLsum = ikLsum + stL.SuccessRate;
    ikRsum = ikRsum + stR.SuccessRate;
    elLsum = elLsum + mL.KeepRatio;
    elRsum = elRsum + mR.KeepRatio;
    if ~col, passCol = passCol + 1; end
end
ev = struct('IKRateL',ikLsum/nEval,'IKRateR',ikRsum/nEval,'ElbowKeepL',elLsum/nEval,'ElbowKeepR',elRsum/nEval,'CollisionPass',passCol/nEval);
end

function visualizeDual(robotL,robotR,Q1,Q2,P1,P2,center,ellABC,meta,fps)
fig = figure("Color","w");
ax = axes(fig); hold(ax,"on");
axis(ax,"equal"); grid(ax,"on");
view(ax,35,20);
xlabel(ax,"X [m]"); ylabel(ax,"Y [m]"); zlabel(ax,"Z [m]");

plotEllipsoid(ax,center,ellABC(1),ellABC(2),ellABC(3));
plot3(ax,P1(:,1),P1(:,2),P1(:,3),"-","Color",[0.1 0.3 0.9],"LineWidth",1.2);
plot3(ax,P2(:,1),P2(:,2),P2(:,3),"-","Color",[0.9 0.2 0.2],"LineWidth",1.2);

show(robotL,Q1(1,:),"Parent",ax,"PreservePlot",true,"Visuals","on","Frames","off");
show(robotR,Q2(1,:),"Parent",ax,"PreservePlot",true,"Visuals","on","Frames","off");

title(ax, sprintf("dual_target_workspace | mode=%s | vL=%.2f vR=%.2f phase=%.2f", meta.Mode, meta.LeftSpeed, meta.RightSpeed, meta.Phase));

TbL = getTransform(robotL,Q1(1,:),"world_base");
TbR = getTransform(robotR,Q2(1,:),"world_base");
basePts = [TbL(1:3,4).'; TbR(1:3,4).'];
lims = computeSceneLimits(P1,P2,center,ellABC,basePts);
setAxesLimits(ax,lims);
rc = rateControl(fps);
k=1; N = size(Q1,1);
while ishandle(fig)
    cla(ax); hold(ax,"on");
    plotEllipsoid(ax,center,ellABC(1),ellABC(2),ellABC(3));
    plot3(ax,P1(:,1),P1(:,2),P1(:,3),"-","Color",[0.1 0.3 0.9],"LineWidth",1.2);
    plot3(ax,P2(:,1),P2(:,2),P2(:,3),"-","Color",[0.9 0.2 0.2],"LineWidth",1.2);
    show(robotL,Q1(k,:),"Parent",ax,"PreservePlot",true,"Visuals","on","Frames","off");
    show(robotR,Q2(k,:),"Parent",ax,"PreservePlot",true,"Visuals","on","Frames","off");
    setAxesLimits(ax,lims);
    drawnow;
    waitfor(rc);
    k = k + 1;
    if k > N, k = 1; end
end
end

function q0 = neighborSeed(Q,k,qHome)
N = size(Q,1);
if k>1 && k<N
    q0 = 0.5*(Q(k-1,:)+Q(k+1,:));
elseif k>1
    q0 = Q(k-1,:);
else
    q0 = qHome;
end
end

function [q, ok] = solveSingleIK(robot, ik, eeName, Tgoal, q0)
q = q0;
ok = false;

if ik.Mode == "inverseKinematics"
    w = [1 1 1 0.005 0.005 0.005]; % almost position-only
    [qCand, solInfo] = ik.Obj(eeName, Tgoal, w, q0);
    q = qCand;
    if isstruct(solInfo) && isfield(solInfo,'ExitFlag')
        ok = solInfo.ExitFlag > 0;
    else
        ok = all(isfinite(qCand));
    end
    if ~ok
        % retry with slightly stronger orientation damping and perturbed seed
        q1 = q0 + 0.08*(2*rand(size(q0))-1);
        [qCand2, info2] = ik.Obj(eeName, Tgoal, [1 1 1 0.001 0.001 0.001], q1);
        q = qCand2;
        if isstruct(info2) && isfield(info2,'ExitFlag')
            ok = info2.ExitFlag > 0;
        else
            ok = all(isfinite(qCand2));
        end
    end
    return;
end

% fallback numeric (slow but robust for tiny-size usage)
obj = @(qq) positionOnlyObjective(robot,qq,eeName,Tgoal(1:3,4).');
opts = optimset('Display','off','MaxIter',80,'TolX',1e-4,'TolFun',1e-4);
q = fminsearch(obj, q0, opts);
ok = all(isfinite(q));
end

function y = softplus(x,beta)
y = (1/beta)*log(1+exp(beta*x));
end

function [col, info] = armArmCapsuleCollision(robot1,q1,bodies1,rad1,robot2,q2,bodies2,rad2,margin,broadMargin)
col = false;
info = struct('Link1','','Link2','','Distance',nan,'Threshold',nan);

caps1 = buildCapsulesAtConfig(robot1,q1,bodies1,rad1);
caps2 = buildCapsulesAtConfig(robot2,q2,bodies2,rad2);
if isempty(caps1) || isempty(caps2), return; end

for i = 1:numel(caps1)
    c1 = caps1(i);
    for j = 1:numel(caps2)
        c2 = caps2(j);

        midDist = norm(c1.mid-c2.mid);
        broadTh = c1.halfLen + c2.halfLen + c1.r + c2.r + margin + broadMargin;
        if midDist > broadTh
            continue;
        end

        d = segmentSegmentDistance(c1.p0,c1.p1,c2.p0,c2.p1);
        th = c1.r + c2.r + margin;
        if d < th
            col = true;
            info.Link1 = c1.name;
            info.Link2 = c2.name;
            info.Distance = d;
            info.Threshold = th;
            return;
        end
    end
end
end

function caps = buildCapsulesAtConfig(robot,q,bodyNames,radMap)
caps = struct('p0',{},'p1',{},'mid',{},'halfLen',{},'r',{},'name',{});
for i = 1:numel(bodyNames)
    bn = bodyNames{i};
    b = getBody(robot,bn);
    if isempty(b.Parent), continue; end
    pn = char(b.Parent.Name);
    T0 = getTransform(robot,q,pn);
    T1 = getTransform(robot,q,bn);
    p0 = T0(1:3,4).';
    p1 = T1(1:3,4).';
    L = norm(p1-p0);
    if L < 1e-7, continue; end

    caps(end+1).p0 = p0; %#ok<AGROW>
    caps(end).p1 = p1;
    caps(end).mid = 0.5*(p0+p1);
    caps(end).halfLen = 0.5*L;
    caps(end).r = getRadiusForBody(radMap,bn);
    caps(end).name = sprintf("%s->%s", pn, bn);
end
end

function d = segmentSegmentDistance(P0,P1,Q0,Q1)
u = P1-P0; v = Q1-Q0; w = P0-Q0;
a=dot(u,u); b=dot(u,v); c=dot(v,v); d0=dot(u,w); e=dot(v,w);
D=a*c-b*b; SMALL=1e-12;

sN=0; sD=D; tN=0; tD=D;
if D < SMALL
    sN=0; sD=1; tN=e; tD=c;
else
    sN=(b*e-c*d0); tN=(a*e-b*d0);
    if sN < 0
        sN=0; tN=e; tD=c;
    elseif sN > sD
        sN=sD; tN=e+b; tD=c;
    end
end

if tN < 0
    tN=0;
    if -d0 < 0
        sN=0;
    elseif -d0 > a
        sN=sD;
    else
        sN=-d0; sD=a;
    end
elseif tN > tD
    tN=tD;
    if (-d0+b) < 0
        sN=0;
    elseif (-d0+b) > a
        sN=sD;
    else
        sN=(-d0+b); sD=a;
    end
end

if abs(sN) < SMALL, sc=0; else, sc=sN/sD; end
if abs(tN) < SMALL, tc=0; else, tc=tN/tD; end

dP = w + sc*u - tc*v;
d = norm(dP);
end

function [p1,p2] = clampInsideEllipsoid(p1,p2,c0,abc,scale)
a=abc(1); b=abc(2); c=abc(3);
p1 = project(p1);
p2 = project(p2);

    function P = project(P)
        X = (P(:,1)-c0(1))/a;
        Y = (P(:,2)-c0(2))/b;
        Z = (P(:,3)-c0(3))/c;
        r2 = X.^2+Y.^2+Z.^2;
        idx = r2 > scale^2;
        if any(idx)
            s = sqrt(r2(idx))/scale;
            X(idx)=X(idx)./s; Y(idx)=Y(idx)./s; Z(idx)=Z(idx)./s;
            P(idx,1)=c0(1)+a*X(idx);
            P(idx,2)=c0(2)+b*Y(idx);
            P(idx,3)=c0(3)+c*Z(idx);
        end
    end
end

function p = samplePointInEllipsoid(center,abc,scale)
a=abc(1)*scale; b=abc(2)*scale; c=abc(3)*scale;
while true
    x=(2*rand()-1)*a; y=(2*rand()-1)*b; z=(2*rand()-1)*c;
    if (x/a)^2 + (y/b)^2 + (z/c)^2 <= 1
        p = center + [x y z];
        return;
    end
end
end

function qn = perturbSeed(q,sigma)
qn = q + sigma*(2*rand(size(q))-1);
end


function lims = computeSceneLimits(P1,P2,center,ellABC,basePts)
allP = [P1; P2; basePts; center + [ellABC(1) ellABC(2) ellABC(3)]; center - [ellABC(1) ellABC(2) ellABC(3)]];
mn = min(allP,[],1);
mx = max(allP,[],1);
pad = [0.20 0.20 0.20];
lims = [mn-pad; mx+pad];
end

function setAxesLimits(ax,lims)
xlim(ax,[lims(1,1), lims(2,1)]);
ylim(ax,[lims(1,2), lims(2,2)]);
zlim(ax,[max(0,lims(1,3)), lims(2,3)]);
end

function [robotL,robotR] = placeDualRobots(robotRaw, baseL, baseR, yawDegL, yawDegR)
yL = deg2rad(yawDegL);
yR = deg2rad(yawDegR);
TL = trvec2tform(baseL) * axang2tform([0 0 1 yL]);
TR = trvec2tform(baseR) * axang2tform([0 0 1 yR]);
robotL = makePlacedRobot(robotRaw, TL);
robotR = makePlacedRobot(robotRaw, TR);
end

function eeName = resolveEEName(robot, eeRaw)
if any(strcmp(robot.BodyNames, eeRaw))
    eeName = eeRaw;
else
    eeName = robot.BodyNames{end};
end
end

function ik = makeIKBackend(robot)
ik = struct();
if exist('inverseKinematics','class') > 0
    ik.Mode = "inverseKinematics";
    ik.Obj = inverseKinematics("RigidBodyTree", robot);
else
    ik.Mode = "numericFallback";
    ik.Obj = [];
end
end

function [shoulderBody, elbowBody] = pickShoulderElbowBodies(robot)
names = string(robot.BodyNames);
shoulderCandidates = ["link2","link_2","shoulder","upperarm","upper_arm"];
elbowCandidates = ["link3","link_3","elbow","forearm","lowerarm","lower_arm"];
shoulderBody=""; elbowBody="";
for c = shoulderCandidates
    idx = find(contains(lower(names), lower(c)),1,'first');
    if ~isempty(idx), shoulderBody = names(idx); break; end
end
for c = elbowCandidates
    idx = find(contains(lower(names), lower(c)),1,'first');
    if ~isempty(idx), elbowBody = names(idx); break; end
end
if shoulderBody=="", shoulderBody = names(min(2,numel(names))); end
if elbowBody=="", elbowBody = names(min(3,numel(names))); end
shoulderBody = char(shoulderBody);
elbowBody = char(elbowBody);
end

function armBodies = pickArmBodies(robot)
n = string(robot.BodyNames);
ln = lower(n);
keep = contains(ln,"link") | contains(ln,"shoulder") | contains(ln,"arm") | contains(ln,"elbow") | ...
       contains(ln,"fore") | contains(ln,"wrist") | contains(ln,"hand") | contains(ln,"tool") | contains(ln,"ee") | contains(ln,"flange");
keep = keep & ~contains(ln,"world_base");
if nnz(keep) < 2
    keep = true(size(n));
    keep(1) = false;
end
armBodies = cellstr(n(keep));
end

function radMap = buildArmCapsuleRadiusMap(bodyNames)
radMap = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(bodyNames)
    s = lower(string(bodyNames{i}));
    r = 0.050;
    if contains(s,"link1") || contains(s,"link2") || contains(s,"upper") || contains(s,"shoulder")
        r = 0.056;
    elseif contains(s,"link5") || contains(s,"link6") || contains(s,"wrist") || contains(s,"hand") || contains(s,"tool")
        r = 0.042;
    end
    radMap(char(bodyNames{i})) = r;
end
end

function r = getRadiusForBody(radMap, bodyName)
if isKey(radMap, bodyName)
    r = radMap(bodyName);
else
    r = 0.045;
end
end

function plotEllipsoid(ax,c0,a,b,c)
[xe,ye,ze] = ellipsoid(c0(1),c0(2),c0(3),a,b,c,30);
s = surf(ax,xe,ye,ze);
s.EdgeAlpha = 0.15;
s.FaceAlpha = 0.05;
s.FaceColor = [0.2 0.7 0.2];
end

function [robot, eeName, qHome] = loadRobotAutoEE(baseDir)
urdfPath = fullfile(baseDir, "m1509.urdf");
assert(isfile(urdfPath), "URDF not found: %s", urdfPath);

urdfToLoad = urdfPath;
txt = string(fileread(urdfPath));
if contains(txt, "package://")
    patched = regexprep(txt, 'package://[^"]*/', 'm1509/');
    tmpUrdf = fullfile(baseDir, "m1509__patched_tmp.urdf");
    fid = fopen(tmpUrdf, "w");
    assert(fid ~= -1);
    fwrite(fid, patched);
    fclose(fid);
    urdfToLoad = tmpUrdf;
end

robot = importrobot(urdfToLoad);
robot.Gravity = [0 0 -9.81];

robot.DataFormat = "struct";
cfg = homeConfiguration(robot);
qHome = [cfg.JointPosition];
robot.DataFormat = "row";

bodies = string(robot.BodyNames);
parents = strings(0);
for i = 1:numel(bodies)
    b = getBody(robot,bodies(i));
    if ~isempty(b.Parent)
        parents(end+1) = string(b.Parent.Name); %#ok<AGROW>
    end
end
leaf = setdiff(bodies, unique(parents), "stable");
eeName = char(leaf(end));
end

function robotPlaced = makePlacedRobot(robotRaw, Tbase)
robotPlaced = rigidBodyTree('DataFormat','row','MaxNumBodies', robotRaw.NumBodies + 1);
robotPlaced.Gravity = robotRaw.Gravity;

b0 = rigidBody("world_base");
j0 = rigidBodyJoint("world_base_joint", "fixed");
setFixedTransform(j0, Tbase);
b0.Joint = j0;
addBody(robotPlaced, b0, robotPlaced.BaseName);

used = containers.Map('KeyType','char','ValueType','logical');
used(char(robotPlaced.BaseName)) = true;
used('world_base') = true;
nameMap = containers.Map('KeyType','char','ValueType','char');

    function newName = uniqueName(oldName)
        newName = oldName;
        if isKey(used,newName)
            k=1;
            while isKey(used,sprintf("%s_%d",oldName,k))
                k = k + 1;
            end
            newName = sprintf("%s_%d",oldName,k);
        end
        used(newName) = true;
    end

for i = 1:numel(robotRaw.Bodies)
    oldBody = robotRaw.Bodies{i};
    oldName = char(oldBody.Name);

    newName = uniqueName(oldName);
    nameMap(oldName) = newName;

    b = copy(oldBody);
    b.Name = newName;

    oldParent = char(oldBody.Parent.Name);
    if strcmp(oldParent, robotRaw.BaseName)
        parentNew = "world_base";
    else
        parentNew = nameMap(oldParent);
    end

    addBody(robotPlaced, b, parentNew);
end
end

function f = positionOnlyObjective(robot,q,eeName,pGoal)
T = getTransform(robot,q,eeName);
f = norm(T(1:3,4).'-pGoal,2);
end

function v = getOpt(opts,key,defaultVal)
if isfield(opts,key)
    v = opts.(key);
else
    v = defaultVal;
end
end
