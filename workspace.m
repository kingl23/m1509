function dual_target_workspace()
% DUAL_TARGET_WORKSPACE (MATLAB R2023a-safe, no unsupported collision APIs)
%
% ✅ What this script does
%  - Loads M1509 URDF (with package:// patch if needed)
%  - Creates TWO robots placed in world:
%      robot1: x = -baseGap/2
%      robot2: x = +baseGap/2, rotated 180deg about Z (faces robot1)
%  - Creates a target ellipsoid region and generates two EE trajectories inside it
%  - Solves IK for robot1; for robot2, uses a "symmetric-like" joint seed (signMap .* q1)
%  - Validates each full trajectory attempt:
%      (1) self-collision via checkCollision(robot,q) (supported)
%      (2) robot-robot collision proxy (Option A): per-link bounding spheres
%          using body origins + radius map: ||p_i - p_j|| >= r_i + r_j + margin
%      (3) z-positive constraint: all bodies except Base + first body must have z >= zMin
%      (4) EE separation constraints (x-side + min EE distance)
%  - Animates infinitely (loops trajectory) until you close the figure
%
% ⚠️ Note
%  - Exact mesh-mesh robot-robot collision is NOT available in your environment
%    (you already hit the API limitations). This uses Option A (conservative bounds).
%
% Folder layout expected:
%   <folder>/
%     dual_target_workspace.m
%     m1509.urdf
%     m1509/ (mesh files, e.g., MF1509_*.dae)

%% ===================== USER PARAMETERS =====================

% --- geometry / placement ---
baseGap = 0.200;                 % [m] distance between bases along X (200mm)

% --- target ellipsoid (semi-axes) and center ---
ellABC    = [0.450, 0.300, 0.300]; % [m] [a b c]
ellCenter = [0.0, 0.600, 0.350];   % [m] center (x between robots, y=0.6m)

% --- animation / sampling ---
Ttotal = 6.0;                    % [s] duration per loop
fps    = 20;                     % [Hz] samples per second
N      = max(2, round(Ttotal*fps));

% --- trajectory constraints (EE-space) ---
xEEKeep   = 0.110;               % [m] robot1 EE x <= -xEEKeep, robot2 EE x >= +xEEKeep
minEEDist = 0.220;               % [m] minimum EE-EE distance

% --- "Option A" robot-robot collision proxy (bounding spheres) ---
sphereMargin      = 0.015;       % [m] extra safety margin added to (r_i+r_j)
minSphereDistance = 0.0;         % [m] additional absolute minimum beyond radii+margin (usually 0)

% --- z-positive constraint (except base + first link body) ---
zMin = 0.005;                    % [m] require z >= zMin for bodies except Base and first body

% --- symmetry preference (joint-space) ---
% signMap defines preferred relation: q2_seed = signMap .* q1
% This is a heuristic "arm-like" symmetry. If it looks off, flip signs per joint.
signMap = [-1, +1, -1, +1, -1, +1];

% --- retry strategy ---
maxTries    = 200;
shrinkStart = 0.40;              % trajectory amplitude factor at attempt=1 (smaller -> easier)
shrinkFloor = 0.12;              % minimum amplitude
shrinkStep  = 0.010;             % shrink decrement per attempt

% --- debug print ---
printEvery = 10;

rng(1); % deterministic retries

%% ===================== PATH SETUP =====================

baseDir = fileparts(mfilename("fullpath"));
oldDir  = pwd;
cleanup = onCleanup(@() cd(oldDir));
cd(baseDir);
addpath(genpath(baseDir));

%% ===================== LOAD ROBOT =====================

[robotRaw, eeName, qHome] = loadRobotAutoEE(baseDir);
fprintf("[dual_target_workspace] EE used: %s\n", eeName);

%% ===================== PLACE TWO ROBOTS IN WORLD =====================

xOff = baseGap/2;
Rz180  = eul2tform([0 0 pi], "XYZ");          % proper rigid rotation
Tbase1 = trvec2tform([-xOff 0 0]);
Tbase2 = trvec2tform([+xOff 0 0]) * Rz180;

robot1 = makePlacedRobot(robotRaw, Tbase1);
robot2 = makePlacedRobot(robotRaw, Tbase2);

% Useful body lists
bodies1 = robot1.BodyNames;
bodies2 = robot2.BodyNames;

% Define "first body" (after base). We'll exclude base + this first body from zMin constraint.
firstBody1 = bodies1{1};
firstBody2 = bodies2{1};
baseName1  = robot1.BaseName; % usually "base"
baseName2  = robot2.BaseName;

%% ===================== IK SETUP =====================

ik1 = inverseKinematics("RigidBodyTree", robot1);
ik2 = inverseKinematics("RigidBodyTree", robot2);

% Keep constant orientation using home EE orientation
Thome = getTransform(robot1, qHome, eeName);
Rdes  = Thome(1:3,1:3);

% IK weights: emphasize position strongly, keep orientation reasonable
wts = [0.7 0.7 0.7 1 1 1];

%% ===================== OPTION A: BUILD PER-BODY SPHERE RADII =====================

% You can tune these if you want more/less conservatism.
% This function assigns a decreasing radius along the kinematic chain.
radiusMap1 = buildRadiusMap(robot1);
radiusMap2 = buildRadiusMap(robot2);

%% ===================== FIND A VALID TRAJECTORY (REJECTION SAMPLING) =====================

fail = struct("EEside",0,"EEdist",0,"IK",0,"SelfCol",0,"SphereCol",0,"Zpos",0);

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

    % Build desired transforms
    Tt1 = zeros(4,4,N);
    Tt2 = zeros(4,4,N);
    for k = 1:N
        Tt1(:,:,k) = [Rdes, p1(k,:).'; 0 0 0 1];
        Tt2(:,:,k) = [Rdes, p2(k,:).'; 0 0 0 1];
    end

    % IK solve:
    % robot1: normal seeded continuation
    % robot2: seed from symmetric-like mapping of robot1 (arm-like)
    q1 = qHome;
    q2 = qHome;
    Q1cand = zeros(N, numel(qHome));
    Q2cand = zeros(N, numel(qHome));

    badIK = false;
    for k = 1:N
        [q1, ~] = ik1(eeName, Tt1(:,:,k), wts, q1);

        q2seed = applySignMap(q1, signMap);
        [q2, ~] = ik2(eeName, Tt2(:,:,k), wts, q2seed);

        if any(~isfinite(q1)) || any(~isfinite(q2))
            badIK = true;
            break;
        end

        Q1cand(k,:) = q1;
        Q2cand(k,:) = q2;
    end
    if badIK
        fail.IK = fail.IK + 1;
        continue;
    end

    % Validate full trajectory:
    [ok, reason] = validateTrajectory( ...
        robot1, Q1cand, bodies1, baseName1, firstBody1, radiusMap1, ...
        robot2, Q2cand, bodies2, baseName2, firstBody2, radiusMap2, ...
        zMin, sphereMargin, minSphereDistance);

    if ok
        Q1 = Q1cand; Q2 = Q2cand;
        fprintf("[dual_target_workspace] ✅ Accepted (attempt %d/%d, shrink=%.3f)\n", attempt, maxTries, shrink);
        break;
    else
        fail.(reason) = fail.(reason) + 1;
        if mod(attempt, printEvery) == 0
            fprintf("[dual_target_workspace] attempt %d/%d failed(%s), shrink=%.3f | EEside=%d EEdist=%d IK=%d Self=%d Sphere=%d Zpos=%d\n", ...
                attempt, maxTries, reason, shrink, ...
                fail.EEside, fail.EEdist, fail.IK, fail.SelfCol, fail.SphereCol, fail.Zpos);
        end
    end
end

if isempty(Q1)
    fprintf("\n[dual_target_workspace] ❌ No valid trajectory found.\n");
    fprintf("Fail stats: EEside=%d EEdist=%d IK=%d Self=%d Sphere=%d Zpos=%d\n", ...
        fail.EEside, fail.EEdist, fail.IK, fail.SelfCol, fail.SphereCol, fail.Zpos);
    fprintf("Tuning order (most effective first):\n");
    fprintf("  1) Increase baseGap (e.g., 0.25 or 0.30)\n");
    fprintf("  2) Reduce sphere radii (buildRadiusMap) OR reduce sphereMargin\n");
    fprintf("  3) Reduce zMin (e.g., 0.005 -> 0.0)\n");
    fprintf("  4) Reduce xEEKeep or minEEDist slightly\n");
    error("Could not find a valid trajectory in %d attempts.", maxTries);
end

%% ===================== VISUALIZE (INFINITE LOOP) =====================

fig = figure("Color","w");
ax  = axes(fig); hold(ax, "on");
axis(ax, "equal"); grid(ax, "on");
xlabel(ax,"X [m]"); ylabel(ax,"Y [m]"); zlabel(ax,"Z [m]");
title(ax, "Dual M1509: Ellipsoid Targets + Symmetry-Preferred IK + Sphere-Bound Collision Avoidance (Looping)");
view(ax, 35, 20);

a = ellABC(1); b = ellABC(2); c = ellABC(3);
plotEllipsoid(ax, ellCenter, a, b, c);
plot3(ax, p1(:,1), p1(:,2), p1(:,3), "-", "LineWidth", 1);
plot3(ax, p2(:,1), p2(:,2), p2(:,3), "-", "LineWidth", 1);

% initial pose
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

function q2seed = applySignMap(q1, signMap)
% Robustly apply signMap to q1 (row vector).
q2seed = q1;
m = min(numel(q1), numel(signMap));
q2seed(1:m) = q1(1:m) .* signMap(1:m);
end

function [p1, p2] = generateTargets(N, center, abc, xEEKeep, shrink)
% Smooth closed curves inside ellipsoid, separated along x.
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

% re-enforce side after clamp
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
    robot1, Q1, bodies1, baseName1, firstBody1, radMap1, ...
    robot2, Q2, bodies2, baseName2, firstBody2, radMap2, ...
    zMin, sphereMargin, minSphereDistance)

N = size(Q1,1);

for k = 1:N
    q1 = Q1(k,:);
    q2 = Q2(k,:);

    % (1) self-collision checks
    if selfCollideSafe(robot1, q1) || selfCollideSafe(robot2, q2)
        ok = false; reason = "SelfCol"; return;
    end

    % (2) z-positive constraint (exclude base + first body)
    if ~zPositiveExcept(robot1, q1, bodies1, baseName1, firstBody1, zMin)
        ok = false; reason = "Zpos"; return;
    end
    if ~zPositiveExcept(robot2, q2, bodies2, baseName2, firstBody2, zMin)
        ok = false; reason = "Zpos"; return;
    end

    % (3) robot-robot collision proxy (Option A): sphere bounds
    if spheresCollide(robot1, q1, bodies1, radMap1, robot2, q2, bodies2, radMap2, sphereMargin, minSphereDistance)
        ok = false; reason = "SphereCol"; return;
    end
end

ok = true; reason = "";
end

function tf = selfCollideSafe(robot, q)
% Avoid hard failures due to name-value differences across installs.
% You will still possibly see the R2022b behavior warning; that's fine.
tf = false;
try
    % Try to silence behavior-change warning if supported
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

function ok = zPositiveExcept(robot, q, bodyNames, baseName, firstBody, zMin)
% Require z >= zMin for all body frame origins except base and firstBody.
for i = 1:numel(bodyNames)
    bn = bodyNames{i};
    if strcmp(bn, baseName) || strcmp(bn, firstBody)
        continue;
    end
    T = getTransform(robot, q, bn);
    if T(3,4) < zMin
        ok = false;
        return;
    end
end
ok = true;
end

function tf = spheresCollide(robot1, q1, bodies1, radMap1, robot2, q2, bodies2, radMap2, margin, minAbs)
% Option A: per-body bounding spheres, centered at body frame origins.
% Collision if any pair distance < (r1+r2+margin) OR < minAbs.

tf = false;

% Precompute origins for speed
P1 = bodyOrigins(robot1, q1, bodies1);
P2 = bodyOrigins(robot2, q2, bodies2);

R1 = radiiForBodies(bodies1, radMap1);
R2 = radiiForBodies(bodies2, radMap2);

for i = 1:size(P1,1)
    for j = 1:size(P2,1)
        d = norm(P1(i,:) - P2(j,:));
        thresh = R1(i) + R2(j) + margin;
        if d < thresh || d < minAbs
            tf = true;
            return;
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
    if isKey(radMap, key)
        R(i) = radMap(key);
    else
        R(i) = 0.06; % fallback default
    end
end
end

function radMap = buildRadiusMap(robot)
% Conservative per-body sphere radii.
% This is a heuristic: larger near base, smaller near wrist.
% Tune these values if it's too strict/too loose.

names = robot.BodyNames;
n = numel(names);

radMap = containers.Map('KeyType','char','ValueType','double');

% radius profile (meters) - safe-ish generic for 6-axis arm
% You can tighten (smaller) to get more feasible paths, or loosen (bigger) for safety.
rBase = 0.14;
rTip  = 0.06;

for i = 1:n
    alpha = (i-1) / max(1,(n-1));
    r = (1-alpha)*rBase + alpha*rTip;
    radMap(names{i}) = r;
end

% Often the very last link (tool flange) is smaller
radMap(names{end}) = min(radMap(names{end}), 0.05);
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

% Auto EE: last leaf body
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
% Build a new rigidBodyTree with a fixed dummy body "world_base" under root,
% then copy bodies from robotRaw with renaming to avoid collisions (e.g., 'base').

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