function out = dual_static_setup_view(opts)
% DUAL_STATIC_SETUP_VIEW
% Static debug viewer for dual-arm placement/workspace/capsule setup.
% (No trajectory loop / no animation)
%
% Usage:
%   out = dual_static_setup_view();
%   out = dual_static_setup_view(struct('Seed',1,'Verbose',true));

if nargin < 1
    opts = struct();
end

seed = getOpt(opts, 'Seed', 1);
rng(seed);
verbose = getOpt(opts, 'Verbose', true);

% Fixed workspace size (absolute requirement)
workspaceSize = [0.450, 0.300, 0.300];

baseLeftXYZ  = getOpt(opts, 'BaseLeftXYZ',  [-0.34 -0.12 0.00]);
baseRightXYZ = getOpt(opts, 'BaseRightXYZ', [+0.34 +0.12 0.00]);
baseYawDegL  = getOpt(opts, 'BaseYawDegL', 0);
baseYawDegR  = getOpt(opts, 'BaseYawDegR', 0);

forwardOffset   = getOpt(opts, 'ForwardOffset', 0.50);
collisionMargin = getOpt(opts, 'CollisionMargin', 0.015);
broadMargin     = getOpt(opts, 'BroadMargin', 0.10);

%% Path setup
baseDir = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

%% Load robot (URDF missing is the only hard error)
[robotRaw, eeAutoName, qHome] = loadRobotAutoEE(baseDir);

%% Place two robots (yaw-only transforms)
[robotL, robotR] = placeDualRobotsYawOnly(robotRaw, baseLeftXYZ, baseRightXYZ, baseYawDegL, baseYawDegR);

% ensure standing gravity
robotL.Gravity = [0 0 -9.81];
robotR.Gravity = [0 0 -9.81];

%% EE selection (safe)
eeL = chooseEEName(robotL, opts, eeAutoName);
eeR = chooseEEName(robotR, opts, eeAutoName);

% Home transforms
TL_ee = getTransform(robotL, qHome, eeL);
TR_ee = getTransform(robotR, qHome, eeR);

%% Workspace center with forward constraint
userCenter = getOpt(opts, 'WorkspaceCenter', []);
if isempty(userCenter)
    center = 0.5*(TL_ee(1:3,4).' + TR_ee(1:3,4).') + [0, +0.50, -0.35];
else
    center = userCenter;
end

minFrontY = max(baseLeftXYZ(2), baseRightXYZ(2)) + forwardOffset;
if center(2) < minFrontY
    if verbose
        fprintf('[dual_static_setup_view] workspaceCenter Y adjusted for forward constraint: %.3f -> %.3f\n', center(2), minFrontY);
    end
    center(2) = minFrontY;
end

frontOK = (center(2) >= minFrontY);

%% Build capsule proxies (same style as runtime collision approximation)
armBodiesL = pickArmBodies(robotL);
armBodiesR = pickArmBodies(robotR);
radMapL = buildArmCapsuleRadiusMap(armBodiesL);
radMapR = buildArmCapsuleRadiusMap(armBodiesR);

capsL = buildCapsulesAtConfig(robotL, qHome, armBodiesL, radMapL);
capsR = buildCapsulesAtConfig(robotR, qHome, armBodiesR, radMapR);

%% Static probe IK (position-priority only)
probePts = makeProbePoints(center);
probe = runProbeIK(robotL, robotR, eeL, eeR, qHome, probePts, verbose);

%% Print diagnostics
if verbose
    fprintf('\n[dual_static_setup_view] baseL=(%.3f %.3f %.3f), baseR=(%.3f %.3f %.3f)\n', baseLeftXYZ, baseRightXYZ);
    fprintf('[dual_static_setup_view] baseYawDeg L/R=(%.1f, %.1f) | gravity=[0 0 -9.81]\n', baseYawDegL, baseYawDegR);
    fprintf('[dual_static_setup_view] workspaceCenter=(%.3f %.3f %.3f), workspaceSize=[%.3f %.3f %.3f]\n', center, workspaceSize);
    fprintf('[dual_static_setup_view] workspace is in front of both bases: %d (required y >= %.3f)\n', frontOK, minFrontY);
    fprintf('[dual_static_setup_view] EE names: L=%s, R=%s\n', eeL, eeR);

    if ~isempty(capsL) && ~isempty(capsR)
        exL = capsL(1); exR = capsR(1);
        narrowTh = exL.r + exR.r + collisionMargin;
        broadTh = narrowTh + broadMargin;
        fprintf('[dual_static_setup_view] narrow-phase threshold = rL + rR + collisionMargin = %.4f + %.4f + %.4f = %.4f\n', ...
            exL.r, exR.r, collisionMargin, narrowTh);
        fprintf('[dual_static_setup_view] broad-phase uses + broadMargin for fast skip: %.4f + %.4f = %.4f\n', ...
            narrowTh, broadMargin, broadTh);
    end
end

%% Draw static figure
fig = figure('Color','w');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 35, 20);
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
title(ax, 'Dual Static Setup: Robots + Workspace + Capsule Collision Proxies');

% workspace ellipsoid (fixed size)
plotEllipsoid(ax, center, workspaceSize(1), workspaceSize(2), workspaceSize(3));

% show robots at home
show(robotL, qHome, 'Parent', ax, 'PreservePlot', true, 'Visuals','on', 'Frames','off');
show(robotR, qHome, 'Parent', ax, 'PreservePlot', true, 'Visuals','on', 'Frames','off');

% base markers
plot3(ax, baseLeftXYZ(1), baseLeftXYZ(2), baseLeftXYZ(3), 'o', 'Color', [0 0 0.8], 'MarkerFaceColor', [0 0 0.8], 'MarkerSize', 6);
plot3(ax, baseRightXYZ(1), baseRightXYZ(2), baseRightXYZ(3), 'o', 'Color', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0], 'MarkerSize', 6);
text(ax, baseLeftXYZ(1), baseLeftXYZ(2), baseLeftXYZ(3)+0.05, sprintf('L base yaw %.1f°', baseYawDegL), 'Color', [0 0 0.8]);
text(ax, baseRightXYZ(1), baseRightXYZ(2), baseRightXYZ(3)+0.05, sprintf('R base yaw %.1f°', baseYawDegR), 'Color', [0.8 0 0]);

% capsule center lines
for i = 1:numel(capsL)
    c = capsL(i);
    plot3(ax, [c.p0(1) c.p1(1)], [c.p0(2) c.p1(2)], [c.p0(3) c.p1(3)], '-', 'Color', [0.1 0.35 0.95], 'LineWidth', 1.6);
end
for i = 1:numel(capsR)
    c = capsR(i);
    plot3(ax, [c.p0(1) c.p1(1)], [c.p0(2) c.p1(2)], [c.p0(3) c.p1(3)], '-', 'Color', [0.95 0.2 0.2], 'LineWidth', 1.6);
end

% show probe points
for i = 1:size(probePts,1)
    plot3(ax, probePts(i,1), probePts(i,2), probePts(i,3), 'ks', 'MarkerSize', 5, 'MarkerFaceColor', [1 0.95 0.2]);
end

legend(ax, {'Workspace (size fixed)', 'Left base', 'Right base', 'Left arm capsule segments', 'Right arm capsule segments', 'Probe points'}, ...
    'Location', 'bestoutside');

annotationTxt = sprintf(['Collision thresholds:\n', ...
    'narrow: d < r_L + r_R + %.3f\n', ...
    'broad skip uses + %.3f'], collisionMargin, broadMargin);
text(ax, center(1), center(2)-workspaceSize(2)-0.2, center(3)+workspaceSize(3)+0.2, annotationTxt, 'FontSize', 9, 'BackgroundColor', 'w');

% auto range to include bases + workspace + probes
lims = computeSceneLimits([baseLeftXYZ; baseRightXYZ], center, workspaceSize, probePts);
xlim(ax, lims(1,:)); ylim(ax, lims(2,:)); zlim(ax, lims(3,:));

%% output struct
out = struct();
out.baseLeftXYZ = baseLeftXYZ;
out.baseRightXYZ = baseRightXYZ;
out.baseYawDegL = baseYawDegL;
out.baseYawDegR = baseYawDegR;
out.workspaceCenter = center;
out.workspaceSize = workspaceSize;
out.eeNameL = eeL;
out.eeNameR = eeR;
out.qHome = qHome;
out.probe = probe;
out.capsules = struct('left',{capsL},'right',{capsR});
out.forwardCheck = frontOK;
out.forwardMinY = minFrontY;
out.figureHandle = fig;

end

%% ============================ helpers ============================
function ee = chooseEEName(robot, opts, eeAuto)
optEE = getOpt(opts, 'EEName', '');
if ~isempty(optEE) && any(strcmp(robot.BodyNames, optEE))
    ee = optEE;
    return;
end
if any(strcmp(robot.BodyNames, 'link_6'))
    ee = 'link_6';
    return;
end
if any(strcmp(robot.BodyNames, eeAuto))
    ee = eeAuto;
else
    ee = robot.BodyNames{end};
end
end

function probe = runProbeIK(robotL, robotR, eeL, eeR, qHome, probePts, verbose)
probe = struct('point',{},'leftOK',{},'rightOK',{},'leftPosErr',{},'rightPosErr',{});

canUseIK = (exist('inverseKinematics','class') > 0);
if canUseIK
    ikL = inverseKinematics('RigidBodyTree', robotL);
    ikR = inverseKinematics('RigidBodyTree', robotR);
    w = [1 1 1 0.005 0.005 0.005]; % position-priority
else
    ikL = []; ikR = []; w = [];
end

for i = 1:size(probePts,1)
    p = probePts(i,:);

    if canUseIK
        Tgoal = [eye(3), p(:); 0 0 0 1];
        [qL, ~] = ikL(eeL, Tgoal, w, qHome);
        [qR, ~] = ikR(eeR, Tgoal, w, qHome);
        TL = getTransform(robotL,qL,eeL);
        TR = getTransform(robotR,qR,eeR);
        errL = norm(TL(1:3,4).' - p, 2);
        errR = norm(TR(1:3,4).' - p, 2);
        okL = isfinite(errL) && errL < 0.08;
        okR = isfinite(errR) && errR < 0.08;
    else
        okL = false; okR = false;
        errL = inf; errR = inf;
    end

    probe(i).point = p;
    probe(i).leftOK = okL;
    probe(i).rightOK = okR;
    probe(i).leftPosErr = errL;
    probe(i).rightPosErr = errR;

    if verbose
        fprintf('[dual_static_setup_view] probe %d @ (%.3f %.3f %.3f) -> L(ok=%d,err=%.4f) R(ok=%d,err=%.4f)\n', ...
            i, p, okL, errL, okR, errR);
    end
end

if verbose && ~canUseIK
    fprintf('[dual_static_setup_view] inverseKinematics class not found; probe IK marked as failed but figure still generated.\n');
end
end

function pts = makeProbePoints(center)
pts = [
    center + [-0.12,  0.00,  0.00];
    center + [+0.12,  0.00,  0.00];
    center + [ 0.00, +0.05,  0.04];
    center + [-0.08, +0.08, -0.04];
    center + [+0.08, -0.02,  0.02]];
end

function lims = computeSceneLimits(basePts, center, abc, probePts)
allP = [basePts; probePts; center + abc; center - abc];
mn = min(allP,[],1);
mx = max(allP,[],1);
pad = [0.20 0.25 0.20];
lims = [mn(1)-pad(1), mx(1)+pad(1); ...
        mn(2)-pad(2), mx(2)+pad(2); ...
        max(0,mn(3)-pad(3)), mx(3)+pad(3)];
end

function [robotL, robotR] = placeDualRobotsYawOnly(robotRaw, baseL, baseR, yawDegL, yawDegR)
yL = deg2rad(yawDegL);
yR = deg2rad(yawDegR);
TL = trvec2tform(baseL) * axang2tform([0 0 1 yL]);
TR = trvec2tform(baseR) * axang2tform([0 0 1 yR]);
robotL = makePlacedRobot(robotRaw, TL);
robotR = makePlacedRobot(robotRaw, TR);
end

function armBodies = pickArmBodies(robot)
n = string(robot.BodyNames);
ln = lower(n);
keep = contains(ln,'link') | contains(ln,'shoulder') | contains(ln,'arm') | contains(ln,'elbow') | ...
       contains(ln,'fore') | contains(ln,'wrist') | contains(ln,'hand') | contains(ln,'tool') | contains(ln,'ee') | contains(ln,'flange');
keep = keep & ~contains(ln,'world_base');
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
    if contains(s,'link1') || contains(s,'link2') || contains(s,'upper') || contains(s,'shoulder')
        r = 0.056;
    elseif contains(s,'link5') || contains(s,'link6') || contains(s,'wrist') || contains(s,'hand') || contains(s,'tool')
        r = 0.042;
    end
    radMap(char(bodyNames{i})) = r;
end
end

function caps = buildCapsulesAtConfig(robot, q, bodyNames, radMap)
caps = struct('p0',{},'p1',{},'r',{},'name',{},'mid',{},'halfLen',{});
for i = 1:numel(bodyNames)
    bn = bodyNames{i};
    b = getBody(robot, bn);
    if isempty(b.Parent)
        continue;
    end
    pn = char(b.Parent.Name);
    T0 = getTransform(robot, q, pn);
    T1 = getTransform(robot, q, bn);
    p0 = T0(1:3,4).';
    p1 = T1(1:3,4).';
    L = norm(p1-p0);
    if L < 1e-7
        continue;
    end

    caps(end+1).p0 = p0; %#ok<AGROW>
    caps(end).p1 = p1;
    caps(end).r = getRadiusForBody(radMap, bn);
    caps(end).name = sprintf('%s->%s', pn, bn);
    caps(end).mid = 0.5*(p0+p1);
    caps(end).halfLen = 0.5*L;
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
s.FaceAlpha = 0.06;
s.FaceColor = [0.2 0.7 0.2];
end

function [robot, eeName, qHome] = loadRobotAutoEE(baseDir)
urdfPath = fullfile(baseDir, 'm1509.urdf');
assert(isfile(urdfPath), 'URDF not found: %s', urdfPath);

urdfToLoad = urdfPath;
txt = string(fileread(urdfPath));
if contains(txt, 'package://')
    patched = regexprep(txt, 'package://[^"]*/', 'm1509/');
    tmpUrdf = fullfile(baseDir, 'm1509__patched_tmp.urdf');
    fid = fopen(tmpUrdf, 'w'); assert(fid ~= -1);
    fwrite(fid, patched);
    fclose(fid);
    urdfToLoad = tmpUrdf;
end

robot = importrobot(urdfToLoad);
robot.Gravity = [0 0 -9.81];

robot.DataFormat = 'struct';
cfg = homeConfiguration(robot);
qHome = [cfg.JointPosition];
robot.DataFormat = 'row';

bodies = string(robot.BodyNames);
parents = strings(0);
for i = 1:numel(bodies)
    b = getBody(robot, bodies(i));
    if ~isempty(b.Parent)
        parents(end+1) = string(b.Parent.Name); %#ok<AGROW>
    end
end
leaf = setdiff(bodies, unique(parents), 'stable');
eeName = char(leaf(end));
end

function robotPlaced = makePlacedRobot(robotRaw, Tbase)
robotPlaced = rigidBodyTree('DataFormat','row','MaxNumBodies', robotRaw.NumBodies + 1);
robotPlaced.Gravity = robotRaw.Gravity;

b0 = rigidBody('world_base');
j0 = rigidBodyJoint('world_base_joint', 'fixed');
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
            k = 1;
            while isKey(used,sprintf('%s_%d', oldName, k))
                k = k + 1;
            end
            newName = sprintf('%s_%d', oldName, k);
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
        parentNew = 'world_base';
    else
        parentNew = nameMap(oldParent);
    end

    addBody(robotPlaced, b, parentNew);
end
end

function v = getOpt(opts,key,defaultVal)
if isfield(opts,key)
    v = opts.(key);
else
    v = defaultVal;
end
end
