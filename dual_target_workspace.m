function dual_target_workspace(opts)
% DUAL_TARGET_WORKSPACE (MATLAB R2023a-safe)
%
% Visualize dual-arm random trajectories in target ellipsoid workspace,
% prioritizing robust arm-arm collision rejection.
%
% Usage:
%   dual_target_workspace();
%   dual_target_workspace(struct("Seed", 5, "NumEvalTraj", 20));

%% ===================== USER PARAMETERS =====================
if nargin < 1
    opts = struct();
end

% --- geometry / placement ---
baseGap = getOpt(opts, "BaseGap", 0.400);                         % [m]

% --- target ellipsoid (semi-axes) and center ---
ellABC    = getOpt(opts, "EllABC",    [0.450, 0.300, 0.300]);     % [m]
ellCenter = getOpt(opts, "EllCenter", [0.0, 0.600, 0.350]);       % [m]

% --- animation / sampling ---
Ttotal = getOpt(opts, "Ttotal", 6.0);                              % [s]
fps    = getOpt(opts, "Fps", 20);                                  % [Hz]
N      = max(2, round(Ttotal*fps));

% --- trajectory constraints (EE-space) ---
xEEKeep   = getOpt(opts, "XEEKeep", 0.110);                        % [m]
minEEDist = getOpt(opts, "MinEEDist", 0.220);                      % [m]

% --- arm-arm capsule collision (priority) ---
armCapsuleMargin = getOpt(opts, "ArmCapsuleMargin", 0.018);        % [m]
armBroadMargin   = getOpt(opts, "ArmBroadMargin", 0.08);           % [m]

% --- existing sphere proxy (secondary backup) ---
sphereMargin      = getOpt(opts, "SphereMargin", 0.012);           % [m]
minSphereDistance = getOpt(opts, "MinSphereDistance", 0.0);        % [m]

% --- ELBOW-UP posture constraint (hard) ---
elbowUpMargin = getOpt(opts, "ElbowUpMargin", 0.030);              % [m]

% --- symmetry preference (joint-space heuristic seed) ---
signMap = getOpt(opts, "SignMap", [-1, +1, -1, +1, -1, +1]);

% --- IK branch exploration strength (elbow-out preference) ---
branchNudgeDeg = getOpt(opts, "BranchNudgeDeg", 25);               % [deg]

% --- retry strategy ---
maxTries    = getOpt(opts, "MaxTries", 200);
shrinkStart = getOpt(opts, "ShrinkStart", 0.40);
shrinkFloor = getOpt(opts, "ShrinkFloor", 0.12);
shrinkStep  = getOpt(opts, "ShrinkStep", 0.010);

% --- debug print ---
printEvery  = getOpt(opts, "PrintEvery", 10);
numEvalTraj = getOpt(opts, "NumEvalTraj", 0); % optional lightweight benchmark
seed        = getOpt(opts, "Seed", 1);
rng(seed);

%% ===================== PATH SETUP =====================

baseDir = fileparts(mfilename("fullpath"));
oldDir  = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

%% ===================== LOAD ROBOT =====================

[robotRaw, eeName, qHome] = loadRobotAutoEE(baseDir);
fprintf("[dual_target_workspace] Seed=%d, EE used: %s\n", seed, eeName);

%% ===================== PLACE TWO ROBOTS =====================

xOff  = baseGap/2;
Tbase1 = trvec2tform([-xOff 0 0]);   % left arm base
Tbase2 = trvec2tform([+xOff 0 0]);   % right arm base

robot1 = makePlacedRobot(robotRaw, Tbase1);
robot2 = makePlacedRobot(robotRaw, Tbase2);

bodies1 = robot1.BodyNames;
bodies2 = robot2.BodyNames;

worldBaseName1 = "world_base";
worldBaseName2 = "world_base";

% Pick shoulder/elbow bodies robustly
[shoulderBody1, elbowBody1] = pickShoulderElbowBodies(robot1);
[shoulderBody2, elbowBody2] = pickShoulderElbowBodies(robot2);
fprintf("[dual_target_workspace] posture bodies: shoulder=%s, elbow=%s\n", shoulderBody1, elbowBody1);

% Arm capsules (priority collision model)
[armBodies1, armBodies2] = pickArmCollisionBodies(robot1, robot2);
radMapCapsule1 = buildArmCapsuleRadiusMap(armBodies1);
radMapCapsule2 = buildArmCapsuleRadiusMap(armBodies2);
fprintf("[dual_target_workspace] arm-arm collision bodies: left=%d, right=%d\n", numel(armBodies1), numel(armBodies2));

%% ===================== IK SETUP =====================

ik1 = inverseKinematics("RigidBodyTree", robot1);
ik2 = inverseKinematics("RigidBodyTree", robot2);

% Keep constant orientation using home EE orientation
Thome = getTransform(robot1, qHome, eeName);
Rdes  = Thome(1:3,1:3);
wts   = [0.7 0.7 0.7 1 1 1];

%% ===================== SECONDARY SPHERE RADII =====================

radiusMap1 = buildRadiusMap(robot1);
radiusMap2 = buildRadiusMap(robot2);

%% ===================== FIND A VALID TRAJECTORY =====================

fail = struct("EEside",0,"EEdist",0,"IK",0,"ElbowUp",0,"ArmArmCol",0,"SelfCol",0,"SphereCol",0);

Q1 = []; Q2 = []; p1 = []; p2 = [];

for attempt = 1:maxTries
    shrink = max(shrinkFloor, shrinkStart - shrinkStep*(attempt-1));

    [p1, p2] = generateTargets(N, ellCenter, ellABC, xEEKeep, shrink);

    % (A) hard EE side separation
    if any(p1(:,1) > -xEEKeep) || any(p2(:,1) < +xEEKeep)
        fail.EEside = fail.EEside + 1;
        continue;
    end

    % (B) EE-EE minimum distance
    if any(vecnorm(p1 - p2, 2, 2) < minEEDist)
        fail.EEdist = fail.EEdist + 1;
        continue;
    end

    % desired transforms
    Tt1 = zeros(4,4,N);
    Tt2 = zeros(4,4,N);
    for k = 1:N
        Tt1(:,:,k) = [Rdes, p1(k,:).'; 0 0 0 1];
        Tt2(:,:,k) = [Rdes, p2(k,:).'; 0 0 0 1];
    end

    q1 = qHome;
    q2 = qHome;

    Q1cand = zeros(N, numel(qHome));
    Q2cand = zeros(N, numel(qHome));

    bad = false;
    badReason = "";

    for k = 1:N
        % --- robot1 IK
        seeds1 = makeIKSeeds(q1, qHome, branchNudgeDeg);
        [q1, ok1, r1] = solveIKPreferElbowOut(ik1, eeName, Tt1(:,:,k), wts, seeds1, ...
            robot1, worldBaseName1, shoulderBody1, elbowBody1, elbowUpMargin, -1);
        if ~ok1
            bad = true; badReason = r1; break;
        end

        % --- robot2 IK
        q2sym  = applySignMap(q1, signMap);
        q2cont = 0.75*q2 + 0.25*q2sym;
        seeds2 = makeIKSeeds(q2cont, qHome, branchNudgeDeg);
        [q2, ok2, r2] = solveIKPreferElbowOut(ik2, eeName, Tt2(:,:,k), wts, seeds2, ...
            robot2, worldBaseName2, shoulderBody2, elbowBody2, elbowUpMargin, +1);
        if ~ok2
            bad = true; badReason = r2; break;
        end

        Q1cand(k,:) = q1;
        Q2cand(k,:) = q2;
    end

    if bad
        fail = bumpFail(fail, badReason);
        if mod(attempt, printEvery) == 0
            printFailLog(attempt, maxTries, badReason, shrink, fail);
        end
        continue;
    end

    % Validate full trajectory (arm-arm first priority)
    [ok, reason] = validateTrajectory( ...
        robot1, Q1cand, bodies1, radiusMap1, worldBaseName1, shoulderBody1, elbowBody1, elbowUpMargin, armBodies1, radMapCapsule1, ...
        robot2, Q2cand, bodies2, radiusMap2, worldBaseName2, shoulderBody2, elbowBody2, elbowUpMargin, armBodies2, radMapCapsule2, ...
        armCapsuleMargin, armBroadMargin, sphereMargin, minSphereDistance);

    if ok
        Q1 = Q1cand; Q2 = Q2cand;
        fprintf("[dual_target_workspace] ✅ Accepted (attempt %d/%d, shrink=%.3f)\n", attempt, maxTries, shrink);
        break;
    else
        fail = bumpFail(fail, reason);
        if mod(attempt, printEvery) == 0
            printFailLog(attempt, maxTries, reason, shrink, fail);
        end
    end
end

if isempty(Q1)
    fprintf("\n[dual_target_workspace] ❌ No valid trajectory found.\n");
    fprintf("Fail stats: EEside=%d EEdist=%d IK=%d ElbowUp=%d ArmArmCol=%d Self=%d Sphere=%d\n", ...
        fail.EEside, fail.EEdist, fail.IK, fail.ElbowUp, fail.ArmArmCol, fail.SelfCol, fail.SphereCol);
    error("Could not find a valid trajectory in %d attempts.", maxTries);
end

%% ===================== OPTIONAL BATCH CHECK =====================
if numEvalTraj > 0
    passCount = quickCollisionPassRate(numEvalTraj, N, ...
        ellCenter, ellABC, xEEKeep, minEEDist, qHome, Rdes, wts, eeName, ...
        ik1, ik2, robot1, robot2, worldBaseName1, worldBaseName2, ...
        shoulderBody1, shoulderBody2, elbowBody1, elbowBody2, elbowUpMargin, ...
        signMap, branchNudgeDeg, armBodies1, armBodies2, radMapCapsule1, radMapCapsule2, armCapsuleMargin, armBroadMargin);
    fprintf("[dual_target_workspace] pass-rate (arm-arm clean) = %d/%d (%.1f%%)\n", ...
        passCount, numEvalTraj, 100*passCount/max(1,numEvalTraj));
end

%% ===================== VISUALIZE (INFINITE LOOP) =====================

fig = figure("Color","w");
ax  = axes(fig); hold(ax, "on");
axis(ax, "equal"); grid(ax, "on");
xlabel(ax,"X [m]"); ylabel(ax,"Y [m]"); zlabel(ax,"Z [m]");
title(ax, "Dual M1509: Random Workspace Trajectory + Arm-Arm Capsule Collision Rejection");
view(ax, 35, 20);

a = ellABC(1); b = ellABC(2); c = ellABC(3);
plotEllipsoid(ax, ellCenter, a, b, c);
plot3(ax, p1(:,1), p1(:,2), p1(:,3), "-", "LineWidth", 1);
plot3(ax, p2(:,1), p2(:,2), p2(:,3), "-", "LineWidth", 1);

show(robot1, Q1(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
show(robot2, Q2(1,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

xlim0 = xlim(ax); ylim0 = ylim(ax); zlim0 = zlim(ax);
rc = rateControl(fps);

k = 1;
while ishandle(fig)
    cla(ax); hold(ax, "on");

    plotEllipsoid(ax, ellCenter, a, b, c);
    plot3(ax, p1(:,1), p1(:,2), p1(:,3), "-", "LineWidth", 1);
    plot3(ax, p2(:,1), p2(:,2), p2(:,3), "-", "LineWidth", 1);

    show(robot1, Q1(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");
    show(robot2, Q2(k,:), "Parent", ax, "PreservePlot", true, "Visuals","on", "Frames","off");

    xlim(ax, xlim0); ylim(ax, ylim0); zlim(ax, zlim0);
    drawnow;
    waitfor(rc);

    k = k + 1;
    if k > N, k = 1; end
end

end

%% =====================================================================
%% ============================ HELPERS =================================
%% =====================================================================

function v = getOpt(opts, key, defaultVal)
if isfield(opts, key)
    v = opts.(key);
else
    v = defaultVal;
end
end

function fail = bumpFail(fail, reason)
if strlength(reason) == 0
    return;
end
r = char(reason);
if isfield(fail, r)
    fail.(r) = fail.(r) + 1;
end
end

function printFailLog(attempt, maxTries, reason, shrink, fail)
fprintf("[dual_target_workspace] attempt %d/%d failed(%s), shrink=%.3f | EEside=%d EEdist=%d IK=%d ElbowUp=%d ArmArm=%d Self=%d Sphere=%d\n", ...
    attempt, maxTries, reason, shrink, ...
    fail.EEside, fail.EEdist, fail.IK, fail.ElbowUp, fail.ArmArmCol, fail.SelfCol, fail.SphereCol);
end

function q2seed = applySignMap(q1, signMap)
q2seed = q1;
m = min(numel(q1), numel(signMap));
q2seed(1:m) = q1(1:m) .* signMap(1:m);
end

function [p1, p2] = generateTargets(N, center, abc, xEEKeep, shrink)
a = abc(1); b = abc(2); c = abc(3);
t = linspace(0, 2*pi, N);

sx = 0.75 * shrink;
sy = 0.75 * shrink;
sz = 0.65 * shrink;

x = (sx*a*cos(t)).';
y = (sy*b*sin(t)).';
z = (sz*c*sin(2*t)).';

C = repmat(center, N, 1);

p1 = [ -(abs(x) + xEEKeep),  y,  z ] + C;
p2 = [ +(abs(x) + xEEKeep),  y,  z ] + C;

[p1, p2] = clampInsideEllipsoid(p1, p2, center, abc, 0.90);

p1(:,1) = min(p1(:,1), -xEEKeep);
p2(:,1) = max(p2(:,1), +xEEKeep);
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

function [ok, reason] = validateTrajectory( ...
    robot1, Q1, bodies1, radMap1, worldBase1, shoulder1, elbow1, elbowUpMargin1, armBodies1, armRadMap1, ...
    robot2, Q2, bodies2, radMap2, worldBase2, shoulder2, elbow2, elbowUpMargin2, armBodies2, armRadMap2, ...
    armCapsuleMargin, armBroadMargin, sphereMargin, minSphereDistance)

N = size(Q1,1);

for k = 1:N
    q1 = Q1(k,:);
    q2 = Q2(k,:);

    % Priority 1: arm-arm collision
    if armArmCapsuleCollision(robot1, q1, armBodies1, armRadMap1, robot2, q2, armBodies2, armRadMap2, armCapsuleMargin, armBroadMargin)
        ok = false; reason = "ArmArmCol"; return;
    end

    % hard elbow-up posture
    if ~elbowUpOK(robot1, q1, worldBase1, shoulder1, elbow1, elbowUpMargin1) || ...
       ~elbowUpOK(robot2, q2, worldBase2, shoulder2, elbow2, elbowUpMargin2)
        ok = false; reason = "ElbowUp"; return;
    end

    % self collisions
    if selfCollideSafe(robot1, q1) || selfCollideSafe(robot2, q2)
        ok = false; reason = "SelfCol"; return;
    end

    % secondary coarse robot-robot sphere proxy (kept for compatibility)
    if spheresCollide(robot1, q1, bodies1, radMap1, robot2, q2, bodies2, radMap2, sphereMargin, minSphereDistance)
        ok = false; reason = "SphereCol"; return;
    end
end

ok = true; reason = "";
end

function tf = elbowUpOK(robot, q, worldBaseName, shoulderBody, elbowBody, margin)
Tbase = getTransform(robot, q, worldBaseName);
Tsh   = getTransform(robot, q, shoulderBody);
Tel   = getTransform(robot, q, elbowBody);
TshL  = Tbase \ Tsh;
TelL  = Tbase \ Tel;
tf = (TelL(3,4) >= TshL(3,4) + margin);
end

function [shoulderBody, elbowBody] = pickShoulderElbowBodies(robot)
names = string(robot.BodyNames);

shoulderCandidates = ["link2","link_2","shoulder","upperarm","upper_arm"];
elbowCandidates    = ["link3","link_3","elbow","forearm","lowerarm","lower_arm"];

shoulderBody = "";
elbowBody    = "";

for c = shoulderCandidates
    idx = find(contains(lower(names), lower(c)), 1, "first");
    if ~isempty(idx), shoulderBody = names(idx); break; end
end
for c = elbowCandidates
    idx = find(contains(lower(names), lower(c)), 1, "first");
    if ~isempty(idx), elbowBody = names(idx); break; end
end

if shoulderBody == ""
    shoulderBody = names(min(2,numel(names)));
end
if elbowBody == ""
    elbowBody = names(min(3,numel(names)));
end

shoulderBody = char(shoulderBody);
elbowBody    = char(elbowBody);
end

function [b1, b2] = pickArmCollisionBodies(robot1, robot2)
b1 = filterArmBodies(robot1.BodyNames);
b2 = filterArmBodies(robot2.BodyNames);

if isempty(b1), b1 = robot1.BodyNames; end
if isempty(b2), b2 = robot2.BodyNames; end
end

function out = filterArmBodies(bodyNames)
if isempty(bodyNames)
    out = {};
    return;
end
n = string(bodyNames);
ln = lower(n);
keywords = ["link","shoulder","upper","arm","elbow","fore","lower","wrist","hand","tool","ee","flange"];
keep = false(size(n));
for i = 1:numel(keywords)
    keep = keep | contains(ln, keywords(i));
end

% remove explicit world base if present
keep = keep & ~contains(ln, "world_base");

if nnz(keep) < 2
    keep = true(size(n));
    keep(1) = false; % usually first is base-like body
end

out = cellstr(n(keep));
end

function seeds = makeIKSeeds(qCont, qHome, branchNudgeDeg)
seeds = {};
seeds{end+1} = qCont;
seeds{end+1} = qHome;

q = qCont;
if numel(q) >= 3
    q3 = q; q3(3) = -q3(3);
    seeds{end+1} = q3;

    d = deg2rad(branchNudgeDeg);
    q2p = q; q2p(2) = q2p(2) + d;
    q2m = q; q2m(2) = q2m(2) - d;
    seeds{end+1} = q2p;
    seeds{end+1} = q2m;

    q23 = q3; q23(2) = q23(2) + d;
    seeds{end+1} = q23;
end
end

function [qBest, ok, reason] = solveIKPreferElbowOut(ik, eeName, Tgoal, wts, seeds, ...
    robot, worldBaseName, shoulderBody, elbowBody, elbowUpMargin, sideSign)

qBest = [];
ok = false;
reason = "IK";

bestScore = inf;
qFallback = [];
fallbackScore = inf;

for i = 1:numel(seeds)
    q0 = seeds{i};
    [q, ~] = ik(eeName, Tgoal, wts, q0);
    if any(~isfinite(q)), continue; end

    if ~elbowUpOK(robot, q, worldBaseName, shoulderBody, elbowBody, elbowUpMargin)
        continue;
    end

    dx = elbowDxLocal(robot, q, worldBaseName, shoulderBody, elbowBody);
    score = -sideSign * dx;

    if (sideSign*dx) >= 0
        if score < bestScore
            bestScore = score;
            qBest = q;
            ok = true;
            reason = "";
        end
    else
        if score < fallbackScore
            fallbackScore = score;
            qFallback = q;
        end
    end
end

if ~ok && ~isempty(qFallback)
    qBest = qFallback;
    ok = true;
    reason = "";
end

if ~ok
    reason = "IK";
end
end

function dx = elbowDxLocal(robot, q, worldBaseName, shoulderBody, elbowBody)
Tbase = getTransform(robot, q, worldBaseName);
Tsh   = getTransform(robot, q, shoulderBody);
Tel   = getTransform(robot, q, elbowBody);
TshL  = Tbase \ Tsh;
TelL  = Tbase \ Tel;
dx = TelL(1,4) - TshL(1,4);
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

function tf = armArmCapsuleCollision(robot1, q1, armBodies1, armRadMap1, robot2, q2, armBodies2, armRadMap2, margin, broadMargin)
tf = false;

caps1 = buildCapsulesAtConfig(robot1, q1, armBodies1, armRadMap1);
caps2 = buildCapsulesAtConfig(robot2, q2, armBodies2, armRadMap2);

if isempty(caps1) || isempty(caps2)
    return;
end

for i = 1:numel(caps1)
    c1 = caps1(i);
    for j = 1:numel(caps2)
        c2 = caps2(j);

        % broad-phase: center distance against conservative bound
        midDist = norm(c1.mid - c2.mid);
        broadTh = c1.halfLen + c2.halfLen + c1.r + c2.r + margin + broadMargin;
        if midDist > broadTh
            continue;
        end

        % narrow-phase: segment-segment distance + radii
        d = segmentSegmentDistance(c1.p0, c1.p1, c2.p0, c2.p1);
        if d < (c1.r + c2.r + margin)
            tf = true;
            return;
        end
    end
end
end

function caps = buildCapsulesAtConfig(robot, q, bodyNames, radMap)
nb = numel(bodyNames);
caps = struct('p0',{},'p1',{},'mid',{},'halfLen',{},'r',{});

for i = 1:nb
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

    segLen = norm(p1 - p0);
    if segLen < 1e-6
        continue;
    end

    caps(end+1).p0 = p0; %#ok<AGROW>
    caps(end).p1 = p1;
    caps(end).mid = 0.5*(p0+p1);
    caps(end).halfLen = 0.5*segLen;
    caps(end).r = getRadiusForBody(radMap, bName);
end
end

function r = getRadiusForBody(radMap, bodyName)
if isKey(radMap, bodyName)
    r = radMap(bodyName);
else
    r = 0.045;
end
end

function d = segmentSegmentDistance(P0, P1, Q0, Q1)
% Robust 3D segment-segment minimum distance (softsurfer-style)
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
    sN = 0;
    sD = 1;
    tN = e;
    tD = c;
else
    sN = (b*e - c*d0);
    tN = (a*e - b*d0);
    if sN < 0
        sN = 0;
        tN = e;
        tD = c;
    elseif sN > sD
        sN = sD;
        tN = e + b;
        tD = c;
    end
end

if tN < 0
    tN = 0;
    if -d0 < 0
        sN = 0;
    elseif -d0 > a
        sN = sD;
    else
        sN = -d0;
        sD = a;
    end
elseif tN > tD
    tN = tD;
    if (-d0 + b) < 0
        sN = 0;
    elseif (-d0 + b) > a
        sN = sD;
    else
        sN = (-d0 + b);
        sD = a;
    end
end

if abs(sN) < SMALL
    sc = 0;
else
    sc = sN / sD;
end
if abs(tN) < SMALL
    tc = 0;
else
    tc = tN / tD;
end

dP = w + sc*u - tc*v;
d = norm(dP);
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
        thresh = R1(i) + R2(j) + margin;
        if d < thresh || d < minAbs
            tf = true; return;
        end
    end
end
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
    if isKey(radMap, key), R(i) = radMap(key);
    else, R(i) = 0.06;
    end
end
end

function radMap = buildRadiusMap(robot)
names = robot.BodyNames;
n = numel(names);

radMap = containers.Map('KeyType','char','ValueType','double');

rBase = 0.14;
rTip  = 0.06;

for i = 1:n
    alpha = (i-1) / max(1,(n-1));
    r = (1-alpha)*rBase + alpha*rTip;
    radMap(names{i}) = r;
end
radMap(names{end}) = min(radMap(names{end}), 0.05);
end

function radMap = buildArmCapsuleRadiusMap(bodyNames)
radMap = containers.Map('KeyType','char','ValueType','double');

for i = 1:numel(bodyNames)
    name = lower(string(bodyNames{i}));
    r = 0.050;
    if contains(name, "shoulder") || contains(name, "upper")
        r = 0.055;
    elseif contains(name, "elbow") || contains(name, "fore") || contains(name, "lower")
        r = 0.050;
    elseif contains(name, "wrist") || contains(name, "hand") || contains(name, "tool") || contains(name, "ee")
        r = 0.040;
    elseif contains(name, "link1") || contains(name, "link2")
        r = 0.055;
    elseif contains(name, "link5") || contains(name, "link6")
        r = 0.042;
    end
    radMap(char(bodyNames{i})) = r;
end
end

function plotEllipsoid(ax, c0, a, b, c)
[xe, ye, ze] = ellipsoid(c0(1), c0(2), c0(3), a, b, c, 30);
s = surf(ax, xe, ye, ze);
s.EdgeAlpha = 0.15;
s.FaceAlpha = 0.05;
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
    fwrite(fid, patched);
    fclose(fid);
    urdfToLoad = tmpUrdf;
end

robot = importrobot(urdfToLoad);
robot.Gravity = [0 0 -9.81];

robot.DataFormat = "struct";
cfgS = homeConfiguration(robot);
qHome = [cfgS.JointPosition];
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

function passCount = quickCollisionPassRate(numEvalTraj, N, ...
    ellCenter, ellABC, xEEKeep, minEEDist, qHome, Rdes, wts, eeName, ...
    ik1, ik2, robot1, robot2, worldBase1, worldBase2, shoulder1, shoulder2, elbow1, elbow2, elbowUpMargin, ...
    signMap, branchNudgeDeg, armBodies1, armBodies2, armRadMap1, armRadMap2, armCapsuleMargin, armBroadMargin)
passCount = 0;

for t = 1:numEvalTraj
    shrink = 0.20 + 0.18*rand();
    [p1, p2] = generateTargets(N, ellCenter, ellABC, xEEKeep, shrink);
    if any(vecnorm(p1-p2,2,2) < minEEDist)
        continue;
    end

    okIK = true;
    q1 = qHome; q2 = qHome;
    Q1 = zeros(N, numel(qHome));
    Q2 = zeros(N, numel(qHome));
    for k = 1:N
        T1 = [Rdes, p1(k,:).'; 0 0 0 1];
        T2 = [Rdes, p2(k,:).'; 0 0 0 1];

        [q1, o1] = solveIKPreferElbowOut(ik1, eeName, T1, wts, makeIKSeeds(q1, qHome, branchNudgeDeg), ...
            robot1, worldBase1, shoulder1, elbow1, elbowUpMargin, -1);
        [q2, o2] = solveIKPreferElbowOut(ik2, eeName, T2, wts, makeIKSeeds(0.75*q2+0.25*applySignMap(q1,signMap), qHome, branchNudgeDeg), ...
            robot2, worldBase2, shoulder2, elbow2, elbowUpMargin, +1);

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
        if armArmCapsuleCollision(robot1, Q1(k,:), armBodies1, armRadMap1, robot2, Q2(k,:), armBodies2, armRadMap2, armCapsuleMargin, armBroadMargin)
            clean = false;
            break;
        end
    end

    if clean
        passCount = passCount + 1;
    end
end
end
