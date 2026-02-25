function dual_target_workspace(opts)
% DUAL_TARGET_WORKSPACE
% Dual-arm random trajectory visualization with robust arm-arm collision rejection.
%
% External dependencies:
%   - MATLAB Robotics System Toolbox (importrobot, inverseKinematics, rigidBodyTree)
%   - m1509.urdf in the same folder as this file
%
% Usage examples:
%   dual_target_workspace();
%   dual_target_workspace(struct("Seed",3,"NumEvalTraj",20,"VerboseCollision",true));

%% ===================== OPTIONS / PARAMETERS =====================
if nargin < 1
    opts = struct();
end

% NOTE (workspace size invariant): semi-axes are preserved unless user explicitly overrides.
ellABCDefault = [0.450, 0.300, 0.300];
ellABC = getOpt(opts, "EllABC", ellABCDefault);

% Workspace POSITION can move (allowed by requirement). Size stays ellABC.
ellCenterNominal = [0.0, 0.600, 0.350];
workspaceOffset  = getOpt(opts, "WorkspaceOffset", [0.00, 0.08, 0.03]);
ellCenter        = ellCenterNominal + workspaceOffset;

% Base placement can be adjusted for structural separation.
baseGap    = getOpt(opts, "BaseGap", 0.52);      % wider than before
baseYSkew  = getOpt(opts, "BaseYSkew", 0.10);    % left/right Y separation
baseZLift  = getOpt(opts, "BaseZLift", 0.00);

Ttotal = getOpt(opts, "Ttotal", 6.0);
fps    = getOpt(opts, "Fps", 20);
N      = max(2, round(Ttotal*fps));

xEEKeep   = getOpt(opts, "XEEKeep", 0.12);
minEEDist = getOpt(opts, "MinEEDist", 0.24);

% Arm-arm capsule collision parameters (highest priority)
armCapsuleMargin = getOpt(opts, "ArmCapsuleMargin", 0.022);
armBroadMargin   = getOpt(opts, "ArmBroadMargin", 0.10);

% Secondary fallback checks
sphereMargin      = getOpt(opts, "SphereMargin", 0.010);
minSphereDistance = getOpt(opts, "MinSphereDistance", 0.0);

% Posture constraints
elbowUpMargin      = getOpt(opts, "ElbowUpMargin", 0.02);
elbowOutMargin     = getOpt(opts, "ElbowOutMargin", 0.018);  % shoulder-relative lateral outward
elbowCenterMargin  = getOpt(opts, "ElbowCenterMargin", 0.025); % body-center-line outward

% IK / sampling strategy
branchNudgeDeg = getOpt(opts, "BranchNudgeDeg", 30);
signMap        = getOpt(opts, "SignMap", [-1, +1, -1, +1, -1, +1]);
maxTries       = getOpt(opts, "MaxTries", 260);
shrinkStart    = getOpt(opts, "ShrinkStart", 0.40);
shrinkFloor    = getOpt(opts, "ShrinkFloor", 0.12);
shrinkStep     = getOpt(opts, "ShrinkStep", 0.008);

% Logging / reproducibility
seed             = getOpt(opts, "Seed", 1);
printEvery       = getOpt(opts, "PrintEvery", 10);
verboseCollision = getOpt(opts, "VerboseCollision", true);
verboseElbow     = getOpt(opts, "VerboseElbow", true);
numEvalTraj      = getOpt(opts, "NumEvalTraj", 0);
rng(seed);

%% ===================== PATH SETUP =====================
baseDir = fileparts(mfilename("fullpath"));
oldDir  = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

%% ===================== LOAD ROBOT =====================
[robotRaw, eeName, qHome] = loadRobotAutoEE(baseDir);
fprintf("[dual_target_workspace] Seed=%d, EE=%s\n", seed, eeName);

%% ===================== PLACE TWO ROBOTS =====================
% World convention (for clarity):
%   +X : left-right direction (centerline split)
%   +Y : forward direction
%   +Z : up
% Left arm base is at negative X, right arm base at positive X.
% Additional Y skew separates working volumes along forward axis.

xOff = baseGap/2;
Tbase1 = trvec2tform([-xOff, -baseYSkew/2, baseZLift]);
Tbase2 = trvec2tform([+xOff, +baseYSkew/2, baseZLift]);

robot1 = makePlacedRobot(robotRaw, Tbase1); % left
robot2 = makePlacedRobot(robotRaw, Tbase2); % right

worldBaseName1 = "world_base";
worldBaseName2 = "world_base";

bodies1 = robot1.BodyNames;
bodies2 = robot2.BodyNames;

[shoulderBody1, elbowBody1] = pickShoulderElbowBodies(robot1);
[shoulderBody2, elbowBody2] = pickShoulderElbowBodies(robot2);

fprintf("[dual_target_workspace] base L=(%.3f,%.3f,%.3f) R=(%.3f,%.3f,%.3f)\n", ...
    -xOff, -baseYSkew/2, baseZLift, +xOff, +baseYSkew/2, baseZLift);
fprintf("[dual_target_workspace] workspace center=(%.3f,%.3f,%.3f), size=[%.3f %.3f %.3f]\n", ...
    ellCenter(1), ellCenter(2), ellCenter(3), ellABC(1), ellABC(2), ellABC(3));

[armBodies1, armBodies2] = pickArmCollisionBodies(robot1, robot2);
armRadMap1 = buildArmCapsuleRadiusMap(armBodies1);
armRadMap2 = buildArmCapsuleRadiusMap(armBodies2);
radiusMap1 = buildRadiusMap(robot1);
radiusMap2 = buildRadiusMap(robot2);

fprintf("[dual_target_workspace] arm bodies left=%d right=%d\n", numel(armBodies1), numel(armBodies2));

%% ===================== IK SETUP =====================
ik1 = inverseKinematics("RigidBodyTree", robot1);
ik2 = inverseKinematics("RigidBodyTree", robot2);

Thome = getTransform(robot1, qHome, eeName);
Rdes = Thome(1:3,1:3);
wts = [0.7 0.7 0.7 1 1 1];

%% ===================== TRAJECTORY SEARCH =====================
fail = struct("EEside",0,"EEdist",0,"IK",0,"ElbowUp",0,"ElbowOut",0,"ArmArmCol",0,"SelfCol",0,"SphereCol",0);

Q1 = []; Q2 = []; p1 = []; p2 = [];
bestLog = struct();

for attempt = 1:maxTries
    shrink = max(shrinkFloor, shrinkStart - shrinkStep*(attempt-1));

    trajOpts = struct();
    trajOpts.Shrink = shrink;
    trajOpts.MinPhaseGap = 0.20;
    [p1cand, p2cand, trajMeta] = generateDualTargets(N, ellCenter, ellABC, xEEKeep, trajOpts);

    if any(p1cand(:,1) > -xEEKeep) || any(p2cand(:,1) < +xEEKeep)
        fail.EEside = fail.EEside + 1;
        continue;
    end

    if any(vecnorm(p1cand - p2cand, 2, 2) < minEEDist)
        fail.EEdist = fail.EEdist + 1;
        continue;
    end

    Tt1 = zeros(4,4,N);
    Tt2 = zeros(4,4,N);
    for k = 1:N
        Tt1(:,:,k) = [Rdes, p1cand(k,:).'; 0 0 0 1];
        Tt2(:,:,k) = [Rdes, p2cand(k,:).'; 0 0 0 1];
    end

    q1 = qHome;
    q2 = qHome;
    Q1cand = zeros(N, numel(qHome));
    Q2cand = zeros(N, numel(qHome));

    bad = false;
    badReason = "";

    for k = 1:N
        seeds1 = makeIKSeeds(q1, qHome, branchNudgeDeg);
        [q1, ok1, r1] = solveIKPreferElbowOut(ik1, eeName, Tt1(:,:,k), wts, seeds1, ...
            robot1, worldBaseName1, shoulderBody1, elbowBody1, elbowUpMargin, elbowOutMargin, elbowCenterMargin, -1);
        if ~ok1
            bad = true; badReason = r1; break;
        end

        q2sym  = applySignMap(q1, signMap);
        q2cont = 0.65*q2 + 0.35*q2sym;
        seeds2 = makeIKSeeds(q2cont, qHome, branchNudgeDeg);
        [q2, ok2, r2] = solveIKPreferElbowOut(ik2, eeName, Tt2(:,:,k), wts, seeds2, ...
            robot2, worldBaseName2, shoulderBody2, elbowBody2, elbowUpMargin, elbowOutMargin, elbowCenterMargin, +1);
        if ~ok2
            bad = true; badReason = r2; break;
        end

        Q1cand(k,:) = q1;
        Q2cand(k,:) = q2;
    end

    if bad
        fail = bumpFail(fail, badReason);
        if mod(attempt, printEvery) == 0
            printFailLog(attempt, maxTries, badReason, shrink, fail, trajMeta);
        end
        continue;
    end

    [ok, reason, diagInfo] = validateTrajectory( ...
        robot1, Q1cand, bodies1, radiusMap1, worldBaseName1, shoulderBody1, elbowBody1, elbowUpMargin, elbowOutMargin, elbowCenterMargin, armBodies1, armRadMap1, ...
        robot2, Q2cand, bodies2, radiusMap2, worldBaseName2, shoulderBody2, elbowBody2, elbowUpMargin, elbowOutMargin, elbowCenterMargin, armBodies2, armRadMap2, ...
        armCapsuleMargin, armBroadMargin, sphereMargin, minSphereDistance);

    if ok
        Q1 = Q1cand; Q2 = Q2cand;
        p1 = p1cand; p2 = p2cand;
        bestLog = diagInfo;
        fprintf("[dual_target_workspace] ✅ Accepted attempt %d/%d shrink=%.3f | vL=%.2f vR=%.2f phase=%.2f\n", ...
            attempt, maxTries, shrink, trajMeta.LeftSpeedScale, trajMeta.RightSpeedScale, trajMeta.PhaseGap);
        break;
    else
        fail = bumpFail(fail, reason);
        if verboseCollision && strcmp(reason, "ArmArmCol") && isfield(diagInfo, "Collision")
            c = diagInfo.Collision;
            fprintf("  [ArmArmCol] t=%d pair=(%s <-> %s) d=%.4f thr=%.4f\n", c.TimeStep, c.Link1, c.Link2, c.Distance, c.Threshold);
        end
        if mod(attempt, printEvery) == 0
            printFailLog(attempt, maxTries, reason, shrink, fail, trajMeta);
        end
    end
end

if isempty(Q1)
    fprintf("\n[dual_target_workspace] ❌ No valid trajectory found.\n");
    fprintf("Fail stats: EEside=%d EEdist=%d IK=%d ElbowUp=%d ElbowOut=%d ArmArmCol=%d Self=%d Sphere=%d\n", ...
        fail.EEside, fail.EEdist, fail.IK, fail.ElbowUp, fail.ElbowOut, fail.ArmArmCol, fail.SelfCol, fail.SphereCol);
    error("Could not find a valid trajectory in %d attempts.", maxTries);
end

%% ===================== ELBOW-OUT METRICS =====================
metrics = evaluateElbowOutMetrics(robot1, Q1, worldBaseName1, shoulderBody1, elbowBody1, -1);
metricsR = evaluateElbowOutMetrics(robot2, Q2, worldBaseName2, shoulderBody2, elbowBody2, +1);
if verboseElbow
    fprintf("[elbow-out] left keep ratio=%.1f%%, mean lateral=%.4f m | right keep ratio=%.1f%%, mean lateral=%.4f m\n", ...
        100*metrics.KeepRatio, metrics.MeanLateral, 100*metricsR.KeepRatio, metricsR.MeanLateral);
end

%% ===================== OPTIONAL BATCH PASS-RATE =====================
if numEvalTraj > 0
    evalResult = quickCollisionPassRate(numEvalTraj, N, ...
        ellCenter, ellABC, xEEKeep, minEEDist, qHome, eeName, Rdes, wts, ...
        ik1, ik2, robot1, robot2, worldBaseName1, worldBaseName2, shoulderBody1, shoulderBody2, elbowBody1, elbowBody2, ...
        elbowUpMargin, elbowOutMargin, elbowCenterMargin, signMap, branchNudgeDeg, ...
        armBodies1, armBodies2, armRadMap1, armRadMap2, armCapsuleMargin, armBroadMargin);

    fprintf("[batch] arm-arm clean pass-rate = %d/%d (%.1f%%), elbow-out keep mean L/R = %.1f%% / %.1f%%\n", ...
        evalResult.PassCount, evalResult.Total, 100*evalResult.PassCount/max(1,evalResult.Total), ...
        100*evalResult.LeftElbowKeepMean, 100*evalResult.RightElbowKeepMean);
end

%% ===================== VISUALIZE =====================
fig = figure("Color","w");
ax  = axes(fig); hold(ax, "on");
axis(ax, "equal"); grid(ax, "on");
xlabel(ax,"X [m]"); ylabel(ax,"Y [m]"); zlabel(ax,"Z [m]");
view(ax, 35, 20);
title(ax, "Dual M1509: Distinct Dual-Trajectory + Arm-Arm Capsule Rejection + Elbow-Out");

a = ellABC(1); b = ellABC(2); c = ellABC(3);
plotEllipsoid(ax, ellCenter, a, b, c);
plot3(ax, p1(:,1), p1(:,2), p1(:,3), "-", "LineWidth", 1.2, "Color", [0.1 0.3 0.9]);
plot3(ax, p2(:,1), p2(:,2), p2(:,3), "-", "LineWidth", 1.2, "Color", [0.9 0.2 0.2]);

show(robot1, Q1(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
show(robot2, Q2(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

if isfield(bestLog, "Meta")
    m = bestLog.Meta;
    txt = sprintf("L: %s %.2fx | R: %s %.2fx | phase=%.2f", m.LeftProfile, m.LeftSpeedScale, m.RightProfile, m.RightSpeedScale, m.PhaseGap);
    text(ax, ellCenter(1), ellCenter(2)-0.35, ellCenter(3)+0.35, txt, "FontSize", 9, "Color", [0.15 0.15 0.15]);
end

xlim0 = xlim(ax); ylim0 = ylim(ax); zlim0 = zlim(ax);
rc = rateControl(fps);
k = 1;
while ishandle(fig)
    cla(ax); hold(ax, "on");
    plotEllipsoid(ax, ellCenter, a, b, c);
    plot3(ax, p1(:,1), p1(:,2), p1(:,3), "-", "LineWidth", 1.2, "Color", [0.1 0.3 0.9]);
    plot3(ax, p2(:,1), p2(:,2), p2(:,3), "-", "LineWidth", 1.2, "Color", [0.9 0.2 0.2]);

    show(robot1, Q1(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
    show(robot2, Q2(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

    xlim(ax, xlim0); ylim(ax, ylim0); zlim(ax, zlim0);
    drawnow;
    waitfor(rc);

    k = k + 1;
    if k > N, k = 1; end
end
end

%% ============================ HELPERS =============================
function v = getOpt(opts, key, defaultVal)
if isfield(opts, key)
    v = opts.(key);
else
    v = defaultVal;
end
end

function fail = bumpFail(fail, reason)
if strlength(reason) == 0, return; end
r = char(reason);
if isfield(fail, r), fail.(r) = fail.(r) + 1; end
end

function printFailLog(attempt, maxTries, reason, shrink, fail, trajMeta)
fprintf("[dual_target_workspace] attempt %d/%d fail(%s) shrink=%.3f | vL=%.2f vR=%.2f phase=%.2f | EEside=%d EEdist=%d IK=%d ElbowUp=%d ElbowOut=%d ArmArm=%d Self=%d Sphere=%d\n", ...
    attempt, maxTries, reason, shrink, trajMeta.LeftSpeedScale, trajMeta.RightSpeedScale, trajMeta.PhaseGap, ...
    fail.EEside, fail.EEdist, fail.IK, fail.ElbowUp, fail.ElbowOut, fail.ArmArmCol, fail.SelfCol, fail.SphereCol);
end

function q2seed = applySignMap(q1, signMap)
q2seed = q1;
m = min(numel(q1), numel(signMap));
q2seed(1:m) = q1(1:m) .* signMap(1:m);
end

function [p1, p2, meta] = generateDualTargets(N, center, abc, xEEKeep, trajOpts)
% Generates different trajectories for two arms by combining:
%  1) different speed scales
%  2) different time profiles (left=sine-ease, right=trapezoid-like)
%  3) phase difference
%  4) different path curvatures/frequencies

shrink = getOpt(trajOpts, "Shrink", 0.35);
minPhaseGap = getOpt(trajOpts, "MinPhaseGap", 0.2);

a = abc(1); b = abc(2); c = abc(3);
t = linspace(0,1,N).';

% distinct time scales
vL = 0.85 + 0.35*rand();
vR = 1.10 + 0.45*rand();
if abs(vL - vR) < 0.12
    vR = vR + 0.18;
end

% phase difference
phiL = 2*pi*rand();
phaseGap = minPhaseGap + 0.25*rand();
phiR = phiL + 2*pi*phaseGap;

% different time profiles
uLraw = mod(vL*t + phiL/(2*pi), 1.0);
uRraw = mod(vR*t + phiR/(2*pi), 1.0);
uL = 0.5 - 0.5*cos(pi*uLraw);        % sine-ease
nuR = trapezoidEase(uRraw, 0.18, 0.68); % trapezoid-like

ampX = 0.72*shrink;
ampY = 0.68*shrink;
ampZ = 0.62*shrink;

% left: smoother lower frequency
xL = -(abs(a*ampX*cos(2*pi*uL)) + xEEKeep);
yL = b*ampY*sin(2*pi*uL + 0.25*pi) - 0.10*b;
zL = c*ampZ*sin(4*pi*uL + 0.30*pi);

% right: different curvature/frequency + phase
xR = +(abs(a*(ampX*0.92)*cos(2*pi*nuR + 0.35*pi)) + xEEKeep);
yR = b*(ampY*0.82)*sin(2*pi*nuR + 1.10*pi) + 0.12*b;
zR = c*(ampZ*0.85)*sin(6*pi*nuR + 0.60*pi);

C = repmat(center, N, 1);
p1 = [xL yL zL] + C;
p2 = [xR yR zR] + C;

[p1, p2] = clampInsideEllipsoid(p1, p2, center, abc, 0.90);
p1(:,1) = min(p1(:,1), -xEEKeep);
p2(:,1) = max(p2(:,1), +xEEKeep);

meta = struct();
meta.LeftSpeedScale = vL;
meta.RightSpeedScale = vR;
meta.PhaseGap = phaseGap;
meta.LeftProfile = "sine-ease";
meta.RightProfile = "trapezoid-ease";
end

function y = trapezoidEase(u, accelFrac, decelStart)
% Piecewise C1 profile: ease-in, constant, ease-out
u = max(0,min(1,u));
y = zeros(size(u));

i1 = u < accelFrac;
i2 = (u >= accelFrac) & (u <= decelStart);
i3 = u > decelStart;

if any(i1)
    s = u(i1)/max(accelFrac,1e-6);
    y(i1) = 0.5*s.^2*accelFrac;
end
if any(i2)
    y0 = 0.5*accelFrac;
    y(i2) = y0 + (u(i2)-accelFrac);
end
if any(i3)
    d = max(1 - decelStart, 1e-6);
    s = (u(i3)-decelStart)/d;
    y3 = (0.5*accelFrac + (decelStart-accelFrac));
    y(i3) = y3 + d*(s - 0.5*s.^2);
end

y = y - min(y);
y = y / max(max(y),1e-9);
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

function [ok, reason, diagInfo] = validateTrajectory( ...
    robot1, Q1, bodies1, radMap1, worldBase1, shoulder1, elbow1, elbowUpMargin1, elbowOutMargin1, elbowCenterMargin1, armBodies1, armRadMap1, ...
    robot2, Q2, bodies2, radMap2, worldBase2, shoulder2, elbow2, elbowUpMargin2, elbowOutMargin2, elbowCenterMargin2, armBodies2, armRadMap2, ...
    armCapsuleMargin, armBroadMargin, sphereMargin, minSphereDistance)

N = size(Q1,1);
diagInfo = struct();

diagInfo.Collision = struct();
diagInfo.Meta = struct();

for k = 1:N
    q1 = Q1(k,:); q2 = Q2(k,:);

    [col, cInfo] = armArmCapsuleCollision(robot1, q1, armBodies1, armRadMap1, robot2, q2, armBodies2, armRadMap2, armCapsuleMargin, armBroadMargin);
    if col
        ok = false; reason = "ArmArmCol";
        cInfo.TimeStep = k;
        diagInfo.Collision = cInfo;
        return;
    end

    if ~elbowUpOK(robot1, q1, worldBase1, shoulder1, elbow1, elbowUpMargin1) || ...
       ~elbowUpOK(robot2, q2, worldBase2, shoulder2, elbow2, elbowUpMargin2)
        ok = false; reason = "ElbowUp"; return;
    end

    if ~elbowOutOK(robot1, q1, shoulder1, elbow1, -1, elbowOutMargin1, elbowCenterMargin1) || ...
       ~elbowOutOK(robot2, q2, shoulder2, elbow2, +1, elbowOutMargin2, elbowCenterMargin2)
        ok = false; reason = "ElbowOut"; return;
    end

    if selfCollideSafe(robot1, q1) || selfCollideSafe(robot2, q2)
        ok = false; reason = "SelfCol"; return;
    end

    if spheresCollide(robot1, q1, bodies1, radMap1, robot2, q2, bodies2, radMap2, sphereMargin, minSphereDistance)
        ok = false; reason = "SphereCol"; return;
    end
end

ok = true; reason = "";
end

function tf = elbowUpOK(robot, q, worldBaseName, shoulderBody, elbowBody, margin)
Tbase = getTransform(robot, q, worldBaseName);
Tsh = getTransform(robot, q, shoulderBody);
Tel = getTransform(robot, q, elbowBody);
TshL = Tbase \ Tsh;
TelL = Tbase \ Tel;
tf = (TelL(3,4) >= TshL(3,4) + margin);
end

function tf = elbowOutOK(robot, q, shoulderBody, elbowBody, sideSign, shoulderRelativeMargin, centerMargin)
Tsh = getTransform(robot, q, shoulderBody);
Tel = getTransform(robot, q, elbowBody);

d = Tel(1:3,4) - Tsh(1:3,4);
outwardLocal = sideSign * d(1);
outwardCenter = sideSign * Tel(1,4);

tf = (outwardLocal >= shoulderRelativeMargin) && (outwardCenter >= centerMargin);
end

function [qBest, ok, reason] = solveIKPreferElbowOut(ik, eeName, Tgoal, wts, seeds, ...
    robot, worldBaseName, shoulderBody, elbowBody, elbowUpMargin, elbowOutMargin, elbowCenterMargin, sideSign)

qBest = [];
ok = false;
reason = "IK";
bestScore = -inf;

for i = 1:numel(seeds)
    q0 = seeds{i};
    [q, ~] = ik(eeName, Tgoal, wts, q0);
    if any(~isfinite(q)), continue; end

    if ~elbowUpOK(robot, q, worldBaseName, shoulderBody, elbowBody, elbowUpMargin)
        continue;
    end

    [lateralLocal, lateralCenter] = elbowOutScalars(robot, q, shoulderBody, elbowBody, sideSign);
    hardOK = (lateralLocal >= elbowOutMargin) && (lateralCenter >= elbowCenterMargin);

    % Prefer stronger elbow-out and centerline separation
    score = 2.0*lateralLocal + 1.4*lateralCenter;
    if hardOK
        score = score + 1.0;
    end

    if score > bestScore
        bestScore = score;
        qBest = q;
        ok = true;
        reason = "";
    end
end

if ~ok
    reason = "IK";
    return;
end

% final hard reject if best candidate still violates elbow-out margins
if ~elbowOutOK(robot, qBest, shoulderBody, elbowBody, sideSign, elbowOutMargin, elbowCenterMargin)
    ok = false;
    reason = "ElbowOut";
end
end

function [localOut, centerOut] = elbowOutScalars(robot, q, shoulderBody, elbowBody, sideSign)
Tsh = getTransform(robot, q, shoulderBody);
Tel = getTransform(robot, q, elbowBody);
d = Tel(1:3,4) - Tsh(1:3,4);
localOut = sideSign * d(1);
centerOut = sideSign * Tel(1,4);
end

function metrics = evaluateElbowOutMetrics(robot, Q, worldBaseName, shoulderBody, elbowBody, sideSign)
N = size(Q,1);
keep = false(N,1);
vals = zeros(N,1);
for k = 1:N
    q = Q(k,:);
    [localOut, ~] = elbowOutScalars(robot, q, shoulderBody, elbowBody, sideSign);
    vals(k) = localOut;
    keep(k) = elbowOutOK(robot, q, shoulderBody, elbowBody, sideSign, 0.0, 0.0);
end
metrics = struct();
metrics.KeepRatio = mean(keep);
metrics.MeanLateral = mean(vals);
metrics.MinLateral = min(vals);
metrics.MaxLateral = max(vals);
metrics.WorldBase = worldBaseName;
end

function seeds = makeIKSeeds(qCont, qHome, branchNudgeDeg)
seeds = {};
seeds{end+1} = qCont;
seeds{end+1} = qHome;

q = qCont;
if numel(q) >= 3
    d = deg2rad(branchNudgeDeg);
    q3 = q; q3(3) = -q3(3); seeds{end+1} = q3;
    q2p = q; q2p(2) = q2p(2) + d; seeds{end+1} = q2p;
    q2m = q; q2m(2) = q2m(2) - d; seeds{end+1} = q2m;
    q23 = q3; q23(2) = q23(2) + d; seeds{end+1} = q23;

    if numel(q) >= 4
        q4p = q; q4p(4) = q4p(4) + 0.6*d; seeds{end+1} = q4p;
        q4m = q; q4m(4) = q4m(4) - 0.6*d; seeds{end+1} = q4m;
    end
end
end

function [collision, info] = armArmCapsuleCollision(robot1, q1, armBodies1, armRadMap1, robot2, q2, armBodies2, armRadMap2, margin, broadMargin)
collision = false;
info = struct('Link1','', 'Link2','', 'Distance',nan, 'Threshold',nan);

caps1 = buildCapsulesAtConfig(robot1, q1, armBodies1, armRadMap1);
caps2 = buildCapsulesAtConfig(robot2, q2, armBodies2, armRadMap2);
if isempty(caps1) || isempty(caps2), return; end

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
            collision = true;
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
% Robust 3D segment-segment distance
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

function tf = selfCollideSafe(robot, q)
tf = false;
try
    tf = any(checkCollision(robot, q, "SkippedSelfCollisions", "parent"), "all");
    return;
catch
end
try
    tf = any(checkCollision(robot, q, "IgnoreSelfCollision", false), "all");
    return;
catch
end
tf = any(checkCollision(robot, q), "all");
end

function tf = spheresCollide(robot1, q1, bodies1, radMap1, robot2, q2, bodies2, radMap2, margin, minAbs)
tf = false;
P1 = bodyOrigins(robot1, q1, bodies1);
P2 = bodyOrigins(robot2, q2, bodies2);
R1 = radiiForBodies(bodies1, radMap1);
R2 = radiiForBodies(bodies2, radMap2);
for i = 1:size(P1,1)
    for j = 1:size(P2,1)
        d = norm(P1(i,:) - P2(j,:));
        th = R1(i)+R2(j)+margin;
        if d < th || d < minAbs
            tf = true; return;
        end
    end
end
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

function [b1, b2] = pickArmCollisionBodies(robot1, robot2)
b1 = filterArmBodies(robot1.BodyNames);
b2 = filterArmBodies(robot2.BodyNames);
if isempty(b1), b1 = robot1.BodyNames; end
if isempty(b2), b2 = robot2.BodyNames; end
end

function out = filterArmBodies(bodyNames)
if isempty(bodyNames), out = {}; return; end
n = string(bodyNames);
ln = lower(n);
keywords = ["link","shoulder","upper","arm","elbow","fore","lower","wrist","hand","tool","ee","flange"];
keep = false(size(n));
for i = 1:numel(keywords)
    keep = keep | contains(ln, keywords(i));
end
keep = keep & ~contains(ln, "world_base");
if nnz(keep) < 2
    keep = true(size(n));
    keep(1) = false;
end
out = cellstr(n(keep));
end

function r = getRadiusForBody(radMap, bodyName)
if isKey(radMap, bodyName), r = radMap(bodyName); else, r = 0.045; end
end

function P = bodyOrigins(robot, q, bodyNames)
nb = numel(bodyNames);
P = zeros(nb,3);
for i = 1:nb
    T = getTransform(robot, q, bodyNames{i});
    P(i,:) = T(1:3,4).';
end
end

function R = radiiForBodies(bodyNames, radMap)
R = zeros(numel(bodyNames),1);
for i = 1:numel(bodyNames)
    key = bodyNames{i};
    if isKey(radMap, key), R(i) = radMap(key); else, R(i)=0.06; end
end
end

function radMap = buildRadiusMap(robot)
names = robot.BodyNames;
n = numel(names);
radMap = containers.Map('KeyType','char','ValueType','double');
rBase = 0.14; rTip = 0.06;
for i = 1:n
    alpha = (i-1)/max(1,n-1);
    radMap(names{i}) = (1-alpha)*rBase + alpha*rTip;
end
radMap(names{end}) = min(radMap(names{end}), 0.05);
end

function radMap = buildArmCapsuleRadiusMap(bodyNames)
radMap = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(bodyNames)
    name = lower(string(bodyNames{i}));
    r = 0.050;
    if contains(name,"shoulder") || contains(name,"upper") || contains(name,"link1") || contains(name,"link2")
        r = 0.057;
    elseif contains(name,"elbow") || contains(name,"fore") || contains(name,"lower") || contains(name,"link3") || contains(name,"link4")
        r = 0.050;
    elseif contains(name,"wrist") || contains(name,"hand") || contains(name,"tool") || contains(name,"ee") || contains(name,"link5") || contains(name,"link6")
        r = 0.042;
    end
    radMap(char(bodyNames{i})) = r;
end
end

function plotEllipsoid(ax, c0, a, b, c)
[xe, ye, ze] = ellipsoid(c0(1), c0(2), c0(3), a, b, c, 28);
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
    fid = fopen(tmpUrdf, "w"); assert(fid ~= -1);
    fwrite(fid, patched); fclose(fid);
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
            while isKey(used, sprintf("%s_%d", oldName, k)), k = k + 1; end
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

function result = quickCollisionPassRate(numEvalTraj, N, ...
    ellCenter, ellABC, xEEKeep, minEEDist, qHome, eeName, Rdes, wts, ...
    ik1, ik2, robot1, robot2, worldBase1, worldBase2, shoulder1, shoulder2, elbow1, elbow2, ...
    elbowUpMargin, elbowOutMargin, elbowCenterMargin, signMap, branchNudgeDeg, ...
    armBodies1, armBodies2, armRadMap1, armRadMap2, armCapsuleMargin, armBroadMargin)

passCount = 0;
leftKeep = zeros(numEvalTraj,1);
rightKeep = zeros(numEvalTraj,1);

for t = 1:numEvalTraj
    trajOpts = struct("Shrink", 0.22 + 0.16*rand(), "MinPhaseGap", 0.22);
    [p1, p2] = generateDualTargets(N, ellCenter, ellABC, xEEKeep, trajOpts);
    if any(vecnorm(p1-p2,2,2) < minEEDist)
        continue;
    end

    q1 = qHome; q2 = qHome;
    Q1 = zeros(N, numel(qHome));
    Q2 = zeros(N, numel(qHome));
    okIK = true;

    for k = 1:N
        T1 = [Rdes, p1(k,:).'; 0 0 0 1];
        T2 = [Rdes, p2(k,:).'; 0 0 0 1];

        [q1, o1] = solveIKPreferElbowOut(ik1, eeName, T1, wts, makeIKSeeds(q1, qHome, branchNudgeDeg), ...
            robot1, worldBase1, shoulder1, elbow1, elbowUpMargin, elbowOutMargin, elbowCenterMargin, -1);
        [q2, o2] = solveIKPreferElbowOut(ik2, eeName, T2, wts, makeIKSeeds(0.65*q2+0.35*applySignMap(q1,signMap), qHome, branchNudgeDeg), ...
            robot2, worldBase2, shoulder2, elbow2, elbowUpMargin, elbowOutMargin, elbowCenterMargin, +1);

        if ~o1 || ~o2
            okIK = false;
            break;
        end
        Q1(k,:) = q1;
        Q2(k,:) = q2;
    end

    if ~okIK
        continue;
    end

    clean = true;
    for k = 1:N
        [c, ~] = armArmCapsuleCollision(robot1, Q1(k,:), armBodies1, armRadMap1, robot2, Q2(k,:), armBodies2, armRadMap2, armCapsuleMargin, armBroadMargin);
        if c
            clean = false;
            break;
        end
    end

    mL = evaluateElbowOutMetrics(robot1, Q1, worldBase1, shoulder1, elbow1, -1);
    mR = evaluateElbowOutMetrics(robot2, Q2, worldBase2, shoulder2, elbow2, +1);
    leftKeep(t) = mL.KeepRatio;
    rightKeep(t) = mR.KeepRatio;

    if clean
        passCount = passCount + 1;
    end
end

result = struct();
result.PassCount = passCount;
result.Total = numEvalTraj;
result.LeftElbowKeepMean = mean(leftKeep(leftKeep>0));
result.RightElbowKeepMean = mean(rightKeep(rightKeep>0));
if ~isfinite(result.LeftElbowKeepMean), result.LeftElbowKeepMean = 0; end
if ~isfinite(result.RightElbowKeepMean), result.RightElbowKeepMean = 0; end
end
