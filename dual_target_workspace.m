function dual_target_workspace(opts)
% DUAL_TARGET_WORKSPACE
% Robust dual-arm trajectory synthesis with:
%   1) high-probability IK convergence (multi-start + continuity seeds)
%   2) distinct left/right trajectories (speed/profile/phase/anchor set)
%   3) elbow-out posture regularization
%   4) arm-arm capsule collision rejection as final strong gate
%
% Dependency notes:
%   - Requires Robotics System Toolbox for importrobot/getTransform/rigidBodyTree.
%   - If inverseKinematics object is unavailable, a numeric fallback solver is used.
%
% Example:
%   dual_target_workspace();
%   dual_target_workspace(struct("Seed",1,"NumEvalTraj",20,"Verbose",true));

if nargin < 1
    opts = struct();
end

%% ------------------------- GLOBAL OPTIONS --------------------------
seed = getOpt(opts, "Seed", 1);
rng(seed);

% Workspace size must stay fixed by requirement.
ellABC = [0.450, 0.300, 0.300];

% Timing / samples
Ttotal = getOpt(opts, "Ttotal", 7.0);
fps    = getOpt(opts, "Fps", 20);
N      = max(40, round(Ttotal*fps));

% Attempts
maxScenarioSelectTries = getOpt(opts, "MaxScenarioSelectTries", 3);
maxTrajTriesPerScenario = getOpt(opts, "MaxTrajTriesPerScenario", 70);

% Geometric guards
xKeep       = getOpt(opts, "XKeep", 0.10);
minEEDist   = getOpt(opts, "MinEEDist", 0.22);

% Posture margins (soft in IK objective, hard in final validation)
elbowOutMarginLocal  = getOpt(opts, "ElbowOutMarginLocal", 0.010);
elbowOutMarginCenter = getOpt(opts, "ElbowOutMarginCenter", 0.015);
elbowUpMargin        = getOpt(opts, "ElbowUpMargin", -0.005); % soft-upward naturalness; relaxed hard gate

% Collision thresholds
capsuleMargin = getOpt(opts, "CapsuleMargin", 0.018);
broadMargin   = getOpt(opts, "BroadMargin", 0.10);

% Logging
verbose = getOpt(opts, "Verbose", true);
verboseCollision = getOpt(opts, "VerboseCollision", true);
numEvalTraj = getOpt(opts, "NumEvalTraj", 0);

%% -------------------------- PATH / LOAD ----------------------------
baseDir = fileparts(mfilename("fullpath"));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

[robotRaw, eeNameRaw, qHome] = loadRobotAutoEE(baseDir);

if verbose
    fprintf("[dual_target_workspace] Seed=%d | N=%d | EE(raw)=%s\n", seed, N, eeNameRaw);
end

%% ------------------------ SCENARIO CANDIDATES ----------------------
% Each scenario moves only BASE / workspace CENTER (size fixed).
scenarioLib = makeScenarioLibrary();
scenarioCount = min(maxScenarioSelectTries, numel(scenarioLib));

bestScenario = struct();
scenarioFound = false;

for sIdx = 1:scenarioCount
    S = scenarioLib(sIdx);

    [robotL, robotR] = placeDualRobots(robotRaw, S.BaseLeftXYZ, S.BaseRightXYZ);

    eeNameL = resolveEEName(robotL, eeNameRaw);
    eeNameR = resolveEEName(robotR, eeNameRaw);

    [shL, elL] = pickShoulderElbowBodies(robotL);
    [shR, elR] = pickShoulderElbowBodies(robotR);

    ikL = makeIKBackend(robotL);
    ikR = makeIKBackend(robotR);

    armBodiesL = pickArmBodies(robotL);
    armBodiesR = pickArmBodies(robotR);
    radL = buildArmCapsuleRadiusMap(armBodiesL);
    radR = buildArmCapsuleRadiusMap(armBodiesR);

    % Fast feasibility probe: find a few reachable anchors on each arm.
    reachOpts = struct( ...
        "AnchorCount", 4, ...
        "MaxSamples", 80, ...
        "XKeep", xKeep, ...
        "ElbowOutMarginLocal", elbowOutMarginLocal, ...
        "ElbowOutMarginCenter", elbowOutMarginCenter, ...
        "ElbowUpMargin", elbowUpMargin);

    [aL, qAL, statL] = findReachableAnchors(robotL, ikL, eeNameL, qHome, S.WorkspaceCenter, ellABC, -1, shL, elL, reachOpts);
    [aR, qAR, statR] = findReachableAnchors(robotR, ikR, eeNameR, qHome, S.WorkspaceCenter, ellABC, +1, shR, elR, reachOpts);

    if verbose
        fprintf("[scenario %d/%d] baseL=(%.2f %.2f %.2f) baseR=(%.2f %.2f %.2f) ws=(%.2f %.2f %.2f) | reach L=%d/%d R=%d/%d\n", ...
            sIdx, scenarioCount, S.BaseLeftXYZ, S.BaseRightXYZ, S.WorkspaceCenter, ...
            statL.Accepted, statL.Tested, statR.Accepted, statR.Tested);
    end

    if numel(aL) >= 4 && numel(aR) >= 4
        scenarioFound = true;
        bestScenario = S;
        bestScenario.robotL = robotL;
        bestScenario.robotR = robotR;
        bestScenario.eeNameL = eeNameL;
        bestScenario.eeNameR = eeNameR;
        bestScenario.ikL = ikL;
        bestScenario.ikR = ikR;
        bestScenario.shL = shL; bestScenario.elL = elL;
        bestScenario.shR = shR; bestScenario.elR = elR;
        bestScenario.armBodiesL = armBodiesL; bestScenario.armBodiesR = armBodiesR;
        bestScenario.radL = radL; bestScenario.radR = radR;
        bestScenario.anchorL = aL; bestScenario.anchorR = aR;
        bestScenario.anchorQL = qAL; bestScenario.anchorQR = qAR;
        break;
    end
end

if ~scenarioFound
    error("No feasible base/workspace scenario found. Please increase scenario library coverage.");
end

if verbose
    fprintf("[selected] baseL=(%.3f %.3f %.3f) baseR=(%.3f %.3f %.3f) wsCenter=(%.3f %.3f %.3f) size=[%.3f %.3f %.3f]\n", ...
        bestScenario.BaseLeftXYZ, bestScenario.BaseRightXYZ, bestScenario.WorkspaceCenter, ellABC);
end

%% ----------------------- TRAJECTORY SYNTHESIS ----------------------
Q1 = []; Q2 = []; P1 = []; P2 = []; trajMeta = struct();
finalDiag = struct();

for attempt = 1:maxTrajTriesPerScenario
    [P1cand, P2cand, meta] = buildDistinctCartesianTraj(bestScenario.anchorL, bestScenario.anchorR, N, bestScenario.WorkspaceCenter, ellABC, xKeep, attempt);

    % Early Cartesian gates
    if any(P1cand(:,1) > -xKeep) || any(P2cand(:,1) < +xKeep)
        continue;
    end
    if any(vecnorm(P1cand - P2cand,2,2) < minEEDist)
        continue;
    end

    % Sequential IK with previous-step continuity seeds.
    ikRunOpts = struct( ...
        "ElbowOutMarginLocal", elbowOutMarginLocal, ...
        "ElbowOutMarginCenter", elbowOutMarginCenter, ...
        "ElbowUpMargin", elbowUpMargin, ...
        "XKeep", xKeep);

    [Q1cand, ikStatL] = solveTrajectoryIK(bestScenario.robotL, bestScenario.ikL, bestScenario.eeNameL, P1cand, ...
        qHome, bestScenario.anchorQL, bestScenario.shL, bestScenario.elL, -1, ikRunOpts);
    [Q2cand, ikStatR] = solveTrajectoryIK(bestScenario.robotR, bestScenario.ikR, bestScenario.eeNameR, P2cand, ...
        qHome, bestScenario.anchorQR, bestScenario.shR, bestScenario.elR, +1, ikRunOpts);

    ikRate = 0.5*(ikStatL.SuccessRate + ikStatR.SuccessRate);
    if ikRate < 0.92
        if verbose && mod(attempt,10)==0
            fprintf("[traj %d/%d] IK low success L=%.1f%% R=%.1f%%\n", attempt, maxTrajTriesPerScenario, 100*ikStatL.SuccessRate, 100*ikStatR.SuccessRate);
        end
        continue;
    end

    % Hard arm-arm collision gate at final stage.
    [isCol, colInfo] = trajectoryArmCollision(bestScenario.robotL, Q1cand, bestScenario.armBodiesL, bestScenario.radL, ...
        bestScenario.robotR, Q2cand, bestScenario.armBodiesR, bestScenario.radR, capsuleMargin, broadMargin);

    if isCol
        if verboseCollision
            fprintf("[traj %d/%d] ArmArm collision t=%d pair=(%s <-> %s) d=%.4f thr=%.4f\n", ...
                attempt, maxTrajTriesPerScenario, colInfo.t, colInfo.linkL, colInfo.linkR, colInfo.d, colInfo.th);
        end
        continue;
    end

    % Final posture metrics
    mL = elbowMetrics(bestScenario.robotL, Q1cand, bestScenario.shL, bestScenario.elL, -1);
    mR = elbowMetrics(bestScenario.robotR, Q2cand, bestScenario.shR, bestScenario.elR, +1);

    if mL.KeepRatio < 0.85 || mR.KeepRatio < 0.85
        continue;
    end

    Q1 = Q1cand; Q2 = Q2cand; P1 = P1cand; P2 = P2cand; trajMeta = meta;
    finalDiag.IKLeft = ikStatL; finalDiag.IKRight = ikStatR;
    finalDiag.ElbowLeft = mL; finalDiag.ElbowRight = mR;
    finalDiag.CollisionPass = true;

    if verbose
        fprintf("[accepted] attempt=%d | IK L/R=%.1f%%/%.1f%% | elbow-out L/R=%.1f%%/%.1f%% | modeL=%s modeR=%s\n", ...
            attempt, 100*ikStatL.SuccessRate, 100*ikStatR.SuccessRate, 100*mL.KeepRatio, 100*mR.KeepRatio, bestScenario.ikL.Mode, bestScenario.ikR.Mode);
    end
    break;
end

if isempty(Q1)
    error("Could not find valid trajectory. IK stage remains infeasible under current configuration.");
end

%% -------------------- OPTIONAL MONTE-CARLO DIAG --------------------
if numEvalTraj > 0
    evalRes = evaluateBatch(bestScenario, qHome, N, numEvalTraj, ellABC, xKeep, minEEDist, ...
        elbowOutMarginLocal, elbowOutMarginCenter, elbowUpMargin, capsuleMargin, broadMargin);

    fprintf("[diagnostics] batch=%d | IK success mean L/R=%.1f%% / %.1f%% | elbow keep L/R=%.1f%% / %.1f%% | arm-arm pass=%.1f%%\n", ...
        numEvalTraj, 100*evalRes.IKRateL, 100*evalRes.IKRateR, 100*evalRes.ElbowKeepL, 100*evalRes.ElbowKeepR, 100*evalRes.CollisionPassRate);
end

%% --------------------------- VISUALIZE -----------------------------
visualizeDual(bestScenario, Q1, Q2, P1, P2, ellABC, trajMeta, fps);

end

%% ==================================================================
%%                             HELPERS
%% ==================================================================

function scenarioLib = makeScenarioLibrary()
% Workspace size fixed. We explore only center position + base placements.
% Values chosen to prioritize reachability first, then collision margin.

scenarioLib = struct([]);

scenarioLib(1).BaseLeftXYZ  = [-0.34, -0.11, 0.00];
scenarioLib(1).BaseRightXYZ = [+0.34, +0.11, 0.00];
scenarioLib(1).WorkspaceCenter = [0.00, 0.58, 0.30];

scenarioLib(2).BaseLeftXYZ  = [-0.31, -0.14, 0.00];
scenarioLib(2).BaseRightXYZ = [+0.31, +0.14, 0.00];
scenarioLib(2).WorkspaceCenter = [0.00, 0.54, 0.28];

scenarioLib(3).BaseLeftXYZ  = [-0.36, -0.08, 0.00];
scenarioLib(3).BaseRightXYZ = [+0.36, +0.08, 0.00];
scenarioLib(3).WorkspaceCenter = [0.00, 0.62, 0.32];
end

function [robotL, robotR] = placeDualRobots(robotRaw, baseL, baseR)
TL = trvec2tform(baseL);
TR = trvec2tform(baseR);
robotL = makePlacedRobot(robotRaw, TL);
robotR = makePlacedRobot(robotRaw, TR);
end

function eeName = resolveEEName(robot, eeNameRaw)
if any(strcmp(robot.BodyNames, eeNameRaw))
    eeName = eeNameRaw;
else
    eeName = robot.BodyNames{end};
end
end

function ik = makeIKBackend(robot)
ik = struct();
ik.Robot = robot;

if exist('inverseKinematics','class') > 0
    ik.Mode = "inverseKinematics";
    ik.Obj  = inverseKinematics("RigidBodyTree", robot);
else
    ik.Mode = "numericFallback";
    ik.Obj  = [];
end
end

function [anchors, anchorQ, stat] = findReachableAnchors(robot, ik, eeName, qHome, center, abc, sideSign, shoulderBody, elbowBody, opts)
needCount = getOpt(opts, "AnchorCount", 4);
maxSamples = getOpt(opts, "MaxSamples", 70);
xKeep = getOpt(opts, "XKeep", 0.10);

outLocal = getOpt(opts, "ElbowOutMarginLocal", 0.0);
outCenter = getOpt(opts, "ElbowOutMarginCenter", 0.0);
upMargin = getOpt(opts, "ElbowUpMargin", -0.02);

anchors = {};
anchorQ = {};

tested = 0;
accepted = 0;
qPrev = qHome;

for i = 1:maxSamples
    tested = tested + 1;

    p = samplePointInEllipsoid(center, abc, 0.80);
    p(1) = sideSign*max(abs(p(1)), xKeep);

    if sideSign < 0
        p(2) = p(2) - 0.04;
    else
        p(2) = p(2) + 0.04;
    end

    seedPool = {qPrev, qHome, perturbSeed(qPrev, 0.20), perturbSeed(qHome, 0.30)};
    [q, ok, info] = solvePoseWithSeeds(robot, ik, eeName, p, qPrev, sideSign, shoulderBody, elbowBody, seedPool);

    if ~ok || info.PosErr > 0.03
        continue;
    end

    [elLocal, elCenter, elUp] = elbowScalars(robot, q, shoulderBody, elbowBody, sideSign);
    if elLocal < outLocal || elCenter < outCenter || elUp < upMargin
        continue;
    end

    accepted = accepted + 1;
    anchors{end+1} = p; %#ok<AGROW>
    anchorQ{end+1} = q; %#ok<AGROW>
    qPrev = q;

    if accepted >= needCount
        break;
    end
end

stat = struct();
stat.Tested = tested;
stat.Accepted = accepted;
end

function [P1, P2, meta] = buildDistinctCartesianTraj(anchorL, anchorR, N, center, abc, xKeep, attempt)
A1 = cell2mat(anchorL(:));
A2 = cell2mat(anchorR(:));

% Ensure loop continuity by closing with first anchor.
A1 = [A1; A1(1,:)];
A2 = [A2; A2(1,:)];

k1 = size(A1,1);
k2 = size(A2,1);
t1 = linspace(0,1,k1);
t2 = linspace(0,1,k2);

t = linspace(0,1,N).';

% Distinct profiles and speeds
speedL = 0.92 + 0.18*rand();
speedR = 1.10 + 0.30*rand();
phase = 0.18 + 0.24*rand();

uL = mod(speedL*t, 1.0);
uR = mod(speedR*t + phase, 1.0);

tauL = 0.5 - 0.5*cos(pi*uL);      % sine-ease
tauR = trapezoidWarp(uR, 0.18, 0.72); % trapezoid-like

% Reorder anchors slightly over attempts to avoid repeated failures.
if mod(attempt,3)==2
    A2 = circshift(A2, 1, 1);
elseif mod(attempt,3)==0
    A1 = circshift(A1, 1, 1);
end

P1 = [interp1(t1, A1(:,1), tauL, 'pchip'), interp1(t1, A1(:,2), tauL, 'pchip'), interp1(t1, A1(:,3), tauL, 'pchip')];
P2 = [interp1(t2, A2(:,1), tauR, 'pchip'), interp1(t2, A2(:,2), tauR, 'pchip'), interp1(t2, A2(:,3), tauR, 'pchip')];

[P1, P2] = clampInsideEllipsoid(P1, P2, center, abc, 0.94);
P1(:,1) = min(P1(:,1), -xKeep);
P2(:,1) = max(P2(:,1), +xKeep);

meta = struct();
meta.LeftSpeed = speedL;
meta.RightSpeed = speedR;
meta.Phase = phase;
meta.LeftProfile = "sine-ease";
meta.RightProfile = "trapezoid-warp";
end

function tau = trapezoidWarp(u, aFrac, dStart)
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
tau = tau ./ max(max(tau),1e-9);
end

function [Q, stat] = solveTrajectoryIK(robot, ik, eeName, P, qHome, anchorQ, shoulderBody, elbowBody, sideSign, opts)
N = size(P,1);
nd = numel(qHome);
Q = zeros(N, nd);

qPrev = qHome;
succ = false(N,1);
posErr = inf(N,1);

for k = 1:N
    p = P(k,:);

    idxA = 1 + mod(k-1, numel(anchorQ));
    qA = anchorQ{idxA};

    seedPool = {
        qPrev, ...
        qA, ...
        qHome, ...
        perturbSeed(qPrev, 0.12), ...
        perturbSeed(qA, 0.18), ...
        perturbSeed(qHome, 0.25)};

    [q, ok, info] = solvePoseWithSeeds(robot, ik, eeName, p, qPrev, sideSign, shoulderBody, elbowBody, seedPool);

    if ok
        Q(k,:) = q;
        qPrev = q;
        succ(k) = true;
        posErr(k) = info.PosErr;
    else
        % continuity-preserving fallback: keep last valid
        if k == 1
            Q(k,:) = qHome;
        else
            Q(k,:) = Q(k-1,:);
        end
        qPrev = Q(k,:);
    end
end

stat = struct();
stat.SuccessMask = succ;
stat.SuccessRate = mean(succ);
stat.MeanPosErr = mean(posErr(succ));
if ~isfinite(stat.MeanPosErr), stat.MeanPosErr = inf; end

% soft recover pass: re-solve failed frames with neighbor seed blending
badIdx = find(~succ);
for i = 1:numel(badIdx)
    k = badIdx(i);
    p = P(k,:);
    qN = neighborSeed(Q, k, qHome);
    seedPool = {qN, qHome, perturbSeed(qN,0.18)};
    [q, ok, info] = solvePoseWithSeeds(robot, ik, eeName, p, qN, sideSign, shoulderBody, elbowBody, seedPool);
    if ok
        Q(k,:) = q;
        succ(k) = true;
        posErr(k) = info.PosErr;
    end
end

stat.SuccessMask = succ;
stat.SuccessRate = mean(succ);
stat.MeanPosErr = mean(posErr(succ));
if ~isfinite(stat.MeanPosErr), stat.MeanPosErr = inf; end
end

function q0 = neighborSeed(Q, k, qHome)
N = size(Q,1);
if k > 1 && k < N
    q0 = 0.5*(Q(k-1,:) + Q(k+1,:));
elseif k > 1
    q0 = Q(k-1,:);
else
    q0 = qHome;
end
end

function [qBest, ok, infoBest] = solvePoseWithSeeds(robot, ik, eeName, pGoal, qPrev, sideSign, shoulderBody, elbowBody, seedPool)
qBest = [];
ok = false;
infoBest = struct('PosErr',inf,'Score',inf);

% Desired orientation: keep home-like but very low orientation weight.
Rdes = eye(3);
Tgoal = [Rdes, pGoal(:); 0 0 0 1];

for i = 1:numel(seedPool)
    q0 = seedPool{i};
    [q, okSolve] = solveSingleIK(robot, ik, eeName, Tgoal, q0);
    if ~okSolve
        continue;
    end

    Tcur = getTransform(robot, q, eeName);
    posErr = norm(Tcur(1:3,4).' - pGoal, 2);

    [elLocal, elCenter, elUp] = elbowScalars(robot, q, shoulderBody, elbowBody, sideSign);

    penOut = softplus(0.012 - elLocal, 40) + softplus(0.016 - elCenter, 40);
    penUp  = softplus(-0.01 - elUp, 30);
    smooth = 0.05*norm(q - qPrev)^2;

    score = 20*posErr + 2.4*penOut + 0.8*penUp + smooth;

    if score < infoBest.Score
        qBest = q;
        infoBest.PosErr = posErr;
        infoBest.Score = score;
        ok = (posErr < 0.035);
    end
end

% Accept very best if moderately close.
if ~ok && ~isempty(qBest)
    ok = (infoBest.PosErr < 0.05);
end
end

function y = softplus(x, beta)
y = (1/beta)*log(1 + exp(beta*x));
end

function [q, ok] = solveSingleIK(robot, ik, eeName, Tgoal, q0)
ok = false;
q = q0;

if ik.Mode == "inverseKinematics"
    % position-priority weights (orientation not strict)
    w = [1 1 1 0.02 0.02 0.02];
    [qCand, solInfo] = ik.Obj(eeName, Tgoal, w, q0);

    q = qCand;
    if isstruct(solInfo) && isfield(solInfo,'ExitFlag')
        ok = solInfo.ExitFlag > 0;
    else
        ok = all(isfinite(qCand));
    end
    return;
end

% Numeric fallback (fminsearch): position only
obj = @(qq) positionOnlyObjective(robot, qq, eeName, Tgoal(1:3,4).');
opts = optimset('Display','off','MaxIter',120);
q = fminsearch(obj, q0, opts);
ok = all(isfinite(q));
end

function [isCol, info] = trajectoryArmCollision(robotL, QL, armBodiesL, radL, robotR, QR, armBodiesR, radR, margin, broadMargin)
isCol = false;
info = struct('t',-1,'linkL','','linkR','','d',nan,'th',nan);
N = size(QL,1);

for k = 1:N
    [col, cinfo] = armArmCapsuleCollision(robotL, QL(k,:), armBodiesL, radL, robotR, QR(k,:), armBodiesR, radR, margin, broadMargin);
    if col
        isCol = true;
        info.t = k;
        info.linkL = cinfo.Link1;
        info.linkR = cinfo.Link2;
        info.d = cinfo.Distance;
        info.th = cinfo.Threshold;
        return;
    end
end
end

function m = elbowMetrics(robot, Q, shoulderBody, elbowBody, sideSign)
N = size(Q,1);
outLocal = zeros(N,1);
outCenter = zeros(N,1);
outUp = zeros(N,1);

for k = 1:N
    [outLocal(k), outCenter(k), outUp(k)] = elbowScalars(robot, Q(k,:), shoulderBody, elbowBody, sideSign);
end

keep = (outLocal > 0) & (outCenter > 0);

m = struct();
m.KeepRatio = mean(keep);
m.LocalMean = mean(outLocal);
m.CenterMean = mean(outCenter);
m.UpMean = mean(outUp);
end

function [outLocal, outCenter, upVal] = elbowScalars(robot, q, shoulderBody, elbowBody, sideSign)
Tsh = getTransform(robot, q, shoulderBody);
Tel = getTransform(robot, q, elbowBody);

d = Tel(1:3,4) - Tsh(1:3,4);
outLocal = sideSign*d(1);
outCenter = sideSign*Tel(1,4);
upVal = d(3);
end

function evalRes = evaluateBatch(S, qHome, N, numEvalTraj, ellABC, xKeep, minEEDist, elbowOutLocal, elbowOutCenter, elbowUpMargin, capsuleMargin, broadMargin)
passIKL = 0;
passIKR = 0;
passCol = 0;
elKeepL = zeros(numEvalTraj,1);
elKeepR = zeros(numEvalTraj,1);

for i = 1:numEvalTraj
    [P1, P2] = buildDistinctCartesianTraj(S.anchorL, S.anchorR, N, S.WorkspaceCenter, ellABC, xKeep, i);

    if any(vecnorm(P1-P2,2,2) < minEEDist)
        continue;
    end

    optsIK = struct("ElbowOutMarginLocal", elbowOutLocal, "ElbowOutMarginCenter", elbowOutCenter, "ElbowUpMargin", elbowUpMargin, "XKeep", xKeep);
    [Q1, stL] = solveTrajectoryIK(S.robotL, S.ikL, S.eeNameL, P1, qHome, S.anchorQL, S.shL, S.elL, -1, optsIK);
    [Q2, stR] = solveTrajectoryIK(S.robotR, S.ikR, S.eeNameR, P2, qHome, S.anchorQR, S.shR, S.elR, +1, optsIK);

    passIKL = passIKL + stL.SuccessRate;
    passIKR = passIKR + stR.SuccessRate;

    mL = elbowMetrics(S.robotL, Q1, S.shL, S.elL, -1);
    mR = elbowMetrics(S.robotR, Q2, S.shR, S.elR, +1);
    elKeepL(i) = mL.KeepRatio;
    elKeepR(i) = mR.KeepRatio;

    [isCol, ~] = trajectoryArmCollision(S.robotL, Q1, S.armBodiesL, S.radL, S.robotR, Q2, S.armBodiesR, S.radR, capsuleMargin, broadMargin);
    if ~isCol
        passCol = passCol + 1;
    end
end

evalRes = struct();
evalRes.IKRateL = passIKL/numEvalTraj;
evalRes.IKRateR = passIKR/numEvalTraj;
evalRes.ElbowKeepL = mean(elKeepL);
evalRes.ElbowKeepR = mean(elKeepR);
evalRes.CollisionPassRate = passCol/numEvalTraj;
end

function visualizeDual(S, Q1, Q2, P1, P2, ellABC, trajMeta, fps)
fig = figure("Color","w");
ax = axes(fig); hold(ax,"on");
axis(ax,"equal"); grid(ax,"on");
view(ax, 35, 20);
xlabel(ax,"X [m]"); ylabel(ax,"Y [m]"); zlabel(ax,"Z [m]");

a = ellABC(1); b = ellABC(2); c = ellABC(3);
plotEllipsoid(ax, S.WorkspaceCenter, a, b, c);
plot3(ax, P1(:,1),P1(:,2),P1(:,3),"-","Color",[0.15 0.35 0.9],"LineWidth",1.2);
plot3(ax, P2(:,1),P2(:,2),P2(:,3),"-","Color",[0.92 0.25 0.2],"LineWidth",1.2);

show(S.robotL, Q1(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
show(S.robotR, Q2(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

title(ax, sprintf("Dual M1509 | L:%s %.2f  R:%s %.2f  phase=%.2f", ...
    trajMeta.LeftProfile, trajMeta.LeftSpeed, trajMeta.RightProfile, trajMeta.RightSpeed, trajMeta.Phase));

xlim0 = xlim(ax); ylim0 = ylim(ax); zlim0 = zlim(ax);
rc = rateControl(fps);

k = 1;
N = size(Q1,1);
while ishandle(fig)
    cla(ax); hold(ax,"on");
    plotEllipsoid(ax, S.WorkspaceCenter, a, b, c);
    plot3(ax, P1(:,1),P1(:,2),P1(:,3),"-","Color",[0.15 0.35 0.9],"LineWidth",1.2);
    plot3(ax, P2(:,1),P2(:,2),P2(:,3),"-","Color",[0.92 0.25 0.2],"LineWidth",1.2);

    show(S.robotL, Q1(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
    show(S.robotR, Q2(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

    xlim(ax, xlim0); ylim(ax, ylim0); zlim(ax, zlim0);
    drawnow;
    waitfor(rc);

    k = k + 1;
    if k > N, k = 1; end
end
end

function [p1, p2] = clampInsideEllipsoid(p1, p2, c0, abc, scale)
a = abc(1); b = abc(2); c = abc(3);
p1 = project(p1);
p2 = project(p2);

    function P = project(P)
        X = (P(:,1)-c0(1))/a;
        Y = (P(:,2)-c0(2))/b;
        Z = (P(:,3)-c0(3))/c;
        r2 = X.^2 + Y.^2 + Z.^2;
        idx = r2 > scale^2;
        if any(idx)
            s = sqrt(r2(idx)) / scale;
            X(idx) = X(idx)./s;
            Y(idx) = Y(idx)./s;
            Z(idx) = Z(idx)./s;
            P(idx,1) = c0(1) + a*X(idx);
            P(idx,2) = c0(2) + b*Y(idx);
            P(idx,3) = c0(3) + c*Z(idx);
        end
    end
end

function p = samplePointInEllipsoid(center, abc, scale)
a = abc(1)*scale; b = abc(2)*scale; c = abc(3)*scale;
while true
    x = (2*rand()-1)*a;
    y = (2*rand()-1)*b;
    z = (2*rand()-1)*c;
    if (x/a)^2 + (y/b)^2 + (z/c)^2 <= 1
        p = center + [x y z];
        return;
    end
end
end

function qn = perturbSeed(q, sigma)
qn = q + sigma*(2*rand(size(q))-1);
end

function [col, info] = armArmCapsuleCollision(robot1, q1, armBodies1, armRadMap1, robot2, q2, armBodies2, armRadMap2, margin, broadMargin)
col = false;
info = struct('Link1','', 'Link2','', 'Distance',nan, 'Threshold',nan);

caps1 = buildCapsulesAtConfig(robot1, q1, armBodies1, armRadMap1);
caps2 = buildCapsulesAtConfig(robot2, q2, armBodies2, armRadMap2);

if isempty(caps1) || isempty(caps2)
    return;
end

for i = 1:numel(caps1)
    c1 = caps1(i);
    for j = 1:numel(caps2)
        c2 = caps2(j);

        midDist = norm(c1.mid - c2.mid);
        broadTh = c1.halfLen + c2.halfLen + c1.r + c2.r + margin + broadMargin;
        if midDist > broadTh
            continue;
        end

        d = segmentSegmentDistance(c1.p0, c1.p1, c2.p0, c2.p1);
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

function caps = buildCapsulesAtConfig(robot, q, bodyNames, radMap)
caps = struct('p0',{},'p1',{},'mid',{},'halfLen',{},'r',{},'name',{});
for i = 1:numel(bodyNames)
    bName = bodyNames{i};
    body = getBody(robot, bName);
    if isempty(body.Parent)
        continue;
    end
    pName = char(body.Parent.Name);

    T0 = getTransform(robot, q, pName);
    T1 = getTransform(robot, q, bName);

    p0 = T0(1:3,4).';
    p1 = T1(1:3,4).';
    segLen = norm(p1-p0);
    if segLen < 1e-7
        continue;
    end

    caps(end+1).p0 = p0; %#ok<AGROW>
    caps(end).p1 = p1;
    caps(end).mid = 0.5*(p0+p1);
    caps(end).halfLen = 0.5*segLen;
    caps(end).r = getRadiusForBody(radMap, bName);
    caps(end).name = sprintf("%s->%s", pName, bName);
end
end

function d = segmentSegmentDistance(P0, P1, Q0, Q1)
u = P1 - P0;
v = Q1 - Q0;
w = P0 - Q0;
a = dot(u,u);
b = dot(u,v);
c = dot(v,v);
d0 = dot(u,w);
e = dot(v,w);
D = a*c - b*b;
SMALL = 1e-12;

sN = 0; sD = D;
tN = 0; tD = D;

if D < SMALL
    sN = 0; sD = 1;
    tN = e; tD = c;
else
    sN = (b*e - c*d0);
    tN = (a*e - b*d0);
    if sN < 0
        sN = 0; tN = e; tD = c;
    elseif sN > sD
        sN = sD; tN = e + b; tD = c;
    end
end

if tN < 0
    tN = 0;
    if -d0 < 0
        sN = 0;
    elseif -d0 > a
        sN = sD;
    else
        sN = -d0; sD = a;
    end
elseif tN > tD
    tN = tD;
    if (-d0 + b) < 0
        sN = 0;
    elseif (-d0 + b) > a
        sN = sD;
    else
        sN = (-d0 + b); sD = a;
    end
end

if abs(sN) < SMALL, sc = 0; else, sc = sN/sD; end
if abs(tN) < SMALL, tc = 0; else, tc = tN/tD; end

dP = w + sc*u - tc*v;
d = norm(dP);
end

function [shoulderBody, elbowBody] = pickShoulderElbowBodies(robot)
names = string(robot.BodyNames);
shoulderCandidates = ["link2","link_2","shoulder","upperarm","upper_arm"];
elbowCandidates = ["link3","link_3","elbow","forearm","lowerarm","lower_arm"];

shoulderBody = ""; elbowBody = "";

for c = shoulderCandidates
    idx = find(contains(lower(names), lower(c)), 1, "first");
    if ~isempty(idx), shoulderBody = names(idx); break; end
end
for c = elbowCandidates
    idx = find(contains(lower(names), lower(c)), 1, "first");
    if ~isempty(idx), elbowBody = names(idx); break; end
end

if shoulderBody == "", shoulderBody = names(min(2,numel(names))); end
if elbowBody == "", elbowBody = names(min(3,numel(names))); end

shoulderBody = char(shoulderBody);
elbowBody = char(elbowBody);
end

function armBodies = pickArmBodies(robot)
names = string(robot.BodyNames);
ln = lower(names);
keep = contains(ln, "link") | contains(ln,"shoulder") | contains(ln,"arm") | contains(ln,"elbow") | ...
       contains(ln,"fore") | contains(ln,"wrist") | contains(ln,"hand") | contains(ln,"tool") | contains(ln,"flange") | contains(ln,"ee");
keep = keep & ~contains(ln, "world_base");
if nnz(keep) < 2
    keep = true(size(names));
    keep(1) = false;
end
armBodies = cellstr(names(keep));
end

function radMap = buildArmCapsuleRadiusMap(bodyNames)
radMap = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(bodyNames)
    name = lower(string(bodyNames{i}));
    r = 0.050;
    if contains(name, "link1") || contains(name,"link2") || contains(name,"shoulder") || contains(name,"upper")
        r = 0.056;
    elseif contains(name, "link3") || contains(name,"link4") || contains(name,"elbow") || contains(name,"fore")
        r = 0.050;
    elseif contains(name, "link5") || contains(name,"link6") || contains(name,"wrist") || contains(name,"hand") || contains(name,"tool")
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

function plotEllipsoid(ax, c0, a, b, c)
[xe, ye, ze] = ellipsoid(c0(1), c0(2), c0(3), a, b, c, 30);
s = surf(ax, xe, ye, ze);
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
    b = getBody(robot, bodies(i));
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
        if isKey(used, newName)
            k = 1;
            while isKey(used, sprintf("%s_%d", oldName, k))
                k = k + 1;
            end
            newName = sprintf("%s_%d", oldName, k);
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


function f = positionOnlyObjective(robot, q, eeName, pGoal)
T = getTransform(robot, q, eeName);
f = norm(T(1:3,4).' - pGoal, 2);
end

function v = getOpt(opts, key, defaultVal)
if isfield(opts, key)
    v = opts.(key);
else
    v = defaultVal;
end
end
