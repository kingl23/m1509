function out = dual_static_setup_view(opts)
% DUAL_STATIC_SETUP_VIEW
% Static setup viewer (no trajectory / no animation)
% - Two upright robots (yaw-only base transforms)
% - Forward workspace with fixed size [0.450 0.300 0.300]
% - IK-derived work posture near workspace center (with elbow-out soft preference)
% - Capsule proxy visualization and margin explanation

if nargin < 1
    opts = struct();
end

% Toolbox requirements (hard error per request)
assert(exist('importrobot','file') > 0, 'Robotics System Toolbox/importrobot is required.');
assert(exist('inverseKinematics','class') > 0, 'Robotics System Toolbox/inverseKinematics is required.');

seed = getOpt(opts, 'Seed', 1);
rng(seed);
verbose = getOpt(opts, 'Verbose', true);

% Fixed workspace size (absolute condition)
workspaceSize = [0.450, 0.300, 0.300];

% Base options (shoulder references, same Y plane, yaw-only)
baseX = abs(getOpt(opts, 'BaseX', 0.34));
baseZ = getOpt(opts, 'BaseZ', 0.00);
baseLeftXYZ  = [-baseX, 0.0, baseZ];
baseRightXYZ = [+baseX, 0.0, baseZ];

yawAngleDeg = getOpt(opts, 'YawAngleDeg', 20);
baseYawDegL  = getOpt(opts, 'BaseYawDegL', +yawAngleDeg);
baseYawDegR  = getOpt(opts, 'BaseYawDegR', -yawAngleDeg);

% Forward / workspace defaults (belly-front region)
forwardOffset = getOpt(opts, 'ForwardOffset', 0.50);
workspaceZRel = getOpt(opts, 'WorkspaceZRel', -0.40);
xKeep         = getOpt(opts, 'XKeep', 0.09);

% Capsule margin diagnostics
collisionMargin = getOpt(opts, 'CollisionMargin', 0.015);
broadMargin     = getOpt(opts, 'BroadMargin', 0.10);

% IK scoring weights
wPos    = getOpt(opts, 'IKPosWeight', 18.0);
wElbow  = getOpt(opts, 'IKElbowWeight', 0.9);
wJDist  = getOpt(opts, 'IKJointDistWeight', 0.02);
wLim    = getOpt(opts, 'IKJointLimitWeight', 0.20);
outMargin = getOpt(opts, 'OutwardMargin', 0.02);

% Weak EE orientation preference (branch bias only)
eeYawOutDeg = getOpt(opts, 'EEYawOutDeg', 15);
eePitchDeg = getOpt(opts, 'EEPitchDeg', -10);
eeRollDeg = getOpt(opts, 'EERollDeg', 0);
ikRotWeight = getOpt(opts, 'IKRotWeight', 0.10);
ikOriScoreWeight = getOpt(opts, 'IKOriScoreWeight', 0.30);

%% Path setup
baseDir = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(baseDir);
addpath(genpath(baseDir));

%% Load and place dual robots
[robotRaw, eeAutoName, qHome] = loadRobotAutoEE(baseDir);

[robotL, robotR] = placeDualRobotsYawOnly(robotRaw, baseLeftXYZ, baseRightXYZ, baseYawDegL, baseYawDegR);
robotL.Gravity = [0 0 -9.81];
robotR.Gravity = [0 0 -9.81];

eeName = chooseEENameCommon(robotL, robotR, opts, eeAutoName);

[shoulderL, elbowL] = pickShoulderElbowBodies(robotL);
[shoulderR, elbowR] = pickShoulderElbowBodies(robotR);

qMinL = extractJointMins(robotL, qHome);
qMaxL = extractJointMaxs(robotL, qHome);
qMinR = extractJointMins(robotR, qHome);
qMaxR = extractJointMaxs(robotR, qHome);

forwardMinY = max(baseLeftXYZ(2), baseRightXYZ(2)) + forwardOffset;

%% Workspace center selection (deterministic belly-front semantics)
userCenter = getOpt(opts, 'WorkspaceCenter', []);
workspaceX = 0.0;
if isempty(userCenter)
    workspaceCenter = [workspaceX, forwardOffset, baseZ + workspaceZRel];
else
    workspaceCenter = userCenter;
    workspaceCenter(1) = 0.0;
end

forwardOk = workspaceCenter(2) >= forwardMinY;
assert(forwardOk, 'Workspace Y must be in front of both shoulders: y >= %.3f.', forwardMinY);
if workspaceCenter(3) >= baseZ
    warning('[dual_static_setup_view] workspaceCenter.z must be below shoulder height (z < %.3f).', baseZ);
end

assert(workspaceCenter(3) < baseZ, 'Workspace Z must be below shoulder (Z<0 relative to shoulder base).');

%% Work posture IK goals near center (slight left/right split)
goalLiftZ = getOpt(opts, 'GoalZOffset', 0.05);
dx = min(0.15, max(0.06, xKeep));
goalL = projectInsideEllipsoid(workspaceCenter + [-dx, 0.00, goalLiftZ], workspaceCenter, workspaceSize, 0.95);
goalR = projectInsideEllipsoid(workspaceCenter + [+dx, 0.00, goalLiftZ], workspaceCenter, workspaceSize, 0.95);

yawL = deg2rad(+eeYawOutDeg);
yawR = deg2rad(-eeYawOutDeg);
pitch = deg2rad(eePitchDeg);
roll = deg2rad(eeRollDeg);
RgoalL = rotmZYX(yawL, pitch, roll);
RgoalR = rotmZYX(yawR, pitch, roll);

repL = solveStaticIKMultiSeed(robotL, eeName, goalL, RgoalL, qHome, qHome, qMinL, qMaxL, shoulderL, elbowL, -1, baseLeftXYZ, wPos, wElbow, wJDist, wLim, outMargin, ikRotWeight, ikOriScoreWeight);
repR = solveStaticIKMultiSeed(robotR, eeName, goalR, RgoalR, qHome, qHome, qMinR, qMaxR, shoulderR, elbowR, +1, baseRightXYZ, wPos, wElbow, wJDist, wLim, outMargin, ikRotWeight, ikOriScoreWeight);

if repL.ok
    qWorkL = repL.q;
else
    qWorkL = qHome;
end
if repR.ok
    qWorkR = repR.q;
else
    qWorkR = qHome;
end

if ~repL.ok
    warning('[dual_static_setup_view] Left IK failed: posErr=%.4f exitFlag=%d outward=%.4f bestScore=%.4f seedTypes=%s. Rendering qHome.', ...
        repL.posErr, repL.exitFlag, repL.elbowOutMetric, repL.score, repL.seedTypesTried);
end
if ~repR.ok
    warning('[dual_static_setup_view] Right IK failed: posErr=%.4f exitFlag=%d outward=%.4f bestScore=%.4f seedTypes=%s. Rendering qHome.', ...
        repR.posErr, repR.exitFlag, repR.elbowOutMetric, repR.score, repR.seedTypesTried);
end

%% Capsule proxies at work posture
armBodiesL = pickArmBodies(robotL);
armBodiesR = pickArmBodies(robotR);
radMapL = buildArmCapsuleRadiusMap(armBodiesL);
radMapR = buildArmCapsuleRadiusMap(armBodiesR);

capsulesL = buildCapsulesAtConfig(robotL, qWorkL, armBodiesL, radMapL);
capsulesR = buildCapsulesAtConfig(robotR, qWorkR, armBodiesR, radMapR);

%% Console logs (required)
if verbose
    fprintf('\n[dual_static_setup_view] baseL=(%.3f %.3f %.3f), baseR=(%.3f %.3f %.3f)\n', baseLeftXYZ, baseRightXYZ);
    fprintf('[dual_static_setup_view] baseYawDeg L/R=(%.1f, %.1f), forwardOffset=%.3f, forwardMinY=%.3f, workspaceZRel=%.3f\n', baseYawDegL, baseYawDegR, forwardOffset, forwardMinY, workspaceCenter(3)-baseZ);
    fprintf('[dual_static_setup_view] EE orient prefs: yawOutDeg=%.1f pitchDeg=%.1f rollDeg=%.1f IKRotWeight=%.3f IKOriScoreWeight=%.3f\n', eeYawOutDeg, eePitchDeg, eeRollDeg, ikRotWeight, ikOriScoreWeight);
    fprintf('[dual_static_setup_view] workspaceCenter=(%.3f %.3f %.3f), workspaceSize=[%.3f %.3f %.3f], forwardOk=%d\n', ...
        workspaceCenter, workspaceSize, forwardOk);
    fprintf('[dual_static_setup_view] EE name=%s\n', eeName);
    fprintf('[dual_static_setup_view] IK L: ok=%d posErr=%.4f oriErr=%.4f outwardMetricWorld=%.4f elbowX=%.4f baseX=%.4f dX=%.4f chosenSeed=%s score=%.4f clampUsed=%d exitFlag=%d\n', ...
        repL.ok, repL.posErr, repL.oriErr, repL.elbowOutMetric, repL.elbowWorldX, repL.baseWorldX, repL.elbowDeltaX, repL.seedType, repL.score, repL.clampUsed, repL.exitFlag);
    fprintf('[dual_static_setup_view] IK R: ok=%d posErr=%.4f oriErr=%.4f outwardMetricWorld=%.4f elbowX=%.4f baseX=%.4f dX=%.4f chosenSeed=%s score=%.4f clampUsed=%d exitFlag=%d\n', ...
        repR.ok, repR.posErr, repR.oriErr, repR.elbowOutMetric, repR.elbowWorldX, repR.baseWorldX, repR.elbowDeltaX, repR.seedType, repR.score, repR.clampUsed, repR.exitFlag);
    fprintf('[dual_static_setup_view] margin notes: narrow threshold = rL + rR + collisionMargin, broad skip uses + broadMargin\n');
end

%% Static figure
fig = figure('Color','w');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 35, 18);
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
title(ax,'Dual Static Setup: Work Posture + Forward Workspace + Capsule Ranges');

plotEllipsoid(ax, workspaceCenter, workspaceSize(1), workspaceSize(2), workspaceSize(3));
plot3(ax, workspaceCenter(1), workspaceCenter(2), workspaceCenter(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
text(ax, workspaceCenter(1), workspaceCenter(2), workspaceCenter(3), '  Workspace (belly front)', 'FontSize', 10);
plot3(ax, [-1 1], [0 0], [0 0], ':', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);

show(robotL, qWorkL, 'Parent', ax, 'PreservePlot', true, 'Visuals','on', 'Frames','off');
show(robotR, qWorkR, 'Parent', ax, 'PreservePlot', true, 'Visuals','on', 'Frames','off');

% Goals
plot3(ax, goalL(1),goalL(2),goalL(3),'o','Color',[0.1 0.3 0.9],'MarkerFaceColor',[0.1 0.3 0.9]);
plot3(ax, goalR(1),goalR(2),goalR(3),'o','Color',[0.9 0.2 0.2],'MarkerFaceColor',[0.9 0.2 0.2]);

% Base markers
plot3(ax, baseLeftXYZ(1),baseLeftXYZ(2),baseLeftXYZ(3),'s','Color',[0 0 0.7],'MarkerFaceColor',[0 0 0.7]);
plot3(ax, baseRightXYZ(1),baseRightXYZ(2),baseRightXYZ(3),'s','Color',[0.7 0 0],'MarkerFaceColor',[0.7 0 0]);

% Capsules: per-link center lines + margin envelope lines
for i = 1:numel(capsulesL)
    c = capsulesL(i);
    drawCapsuleSwept(ax, c.p0, c.p1, c.r, [0.1 0.35 0.95], 0.20);
    drawCapsuleSwept(ax, c.p0, c.p1, c.r + collisionMargin, [0.55 0.75 1.0], 0.08);
end
for i = 1:numel(capsulesR)
    c = capsulesR(i);
    drawCapsuleSwept(ax, c.p0, c.p1, c.r, [0.95 0.2 0.2], 0.20);
    drawCapsuleSwept(ax, c.p0, c.p1, c.r + collisionMargin, [1.0 0.75 0.75], 0.08);
end

for i = 1:min(3,numel(capsulesL))
    c = capsulesL(i);
    text(ax,c.mid(1),c.mid(2),c.mid(3),sprintf('r=%.3f',c.r),'Color',[0.1 0.35 0.95],'FontSize',8);
end
for i = 1:min(3,numel(capsulesR))
    c = capsulesR(i);
    text(ax,c.mid(1),c.mid(2),c.mid(3),sprintf('r=%.3f',c.r),'Color',[0.95 0.2 0.2],'FontSize',8);
end

% Threshold example text
if ~isempty(capsulesL) && ~isempty(capsulesR)
    exL = capsulesL(1); exR = capsulesR(1);
    narrowTh = exL.r + exR.r + collisionMargin;
    broadTh = narrowTh + broadMargin;
    thTxt = sprintf(['Example pair threshold:\n',' narrow: d < rL + rR + collisionMargin\n',' values: %.4f + %.4f + %.4f = %.4f\n',' broad skip: narrow + broadMargin = %.4f + %.4f = %.4f\n',' Base is shoulder reference (z=0), workspace is below shoulder.'], ...
        exL.r, exR.r, collisionMargin, narrowTh, narrowTh, broadMargin, broadTh);
    text(ax, workspaceCenter(1), workspaceCenter(2)-workspaceSize(2)-0.22, workspaceCenter(3)+workspaceSize(3)+0.15, thTxt, 'BackgroundColor','w', 'FontSize',8);
end

legend(ax, {'Workspace ellipsoid (fixed size)','Goal L','Goal R','Base L (shoulder ref)','Base R (shoulder ref)','Left capsules (core+margin)','Right capsules (core+margin)'}, ...
    'Location','bestoutside');

% Auto-limits include bases + workspace + goals + capsule endpoints
lims = computeStaticLimits(baseLeftXYZ, baseRightXYZ, workspaceCenter, workspaceSize, goalL, goalR, capsulesL, capsulesR);
xlim(ax, lims(1,:)); ylim(ax, lims(2,:)); zlim(ax, [min(-0.70, workspaceCenter(3)-0.35), 1.2]);

%% Output struct
out = struct();
out.workspaceCenter = workspaceCenter;
out.workspaceSize = workspaceSize;
out.baseLeftXYZ = baseLeftXYZ;
out.baseRightXYZ = baseRightXYZ;
out.baseYawDegL = baseYawDegL;
out.baseYawDegR = baseYawDegR;
out.eeName = eeName;
out.qHome = qHome;
out.qWorkL = qWorkL;
out.qWorkR = qWorkR;
out.goalL = goalL;
out.goalR = goalR;
out.capsulesL = capsulesL;
out.capsulesR = capsulesR;
out.forwardOk = forwardOk;
out.forwardMinY = forwardMinY;
out.ikReportL = repL;
out.ikReportR = repR;
out.outwardMetricL = repL.elbowOutMetric;
out.outwardMetricR = repR.elbowOutMetric;
out.figureHandle = fig;

end

%% --------------------------- local helpers ---------------------------

function report = solveStaticIKMultiSeed(robot, eeName, pGoal, Rgoal, qSeed, qRef, qMin, qMax, shoulderBody, elbowBody, sideSign, baseWorldXYZ, wPos, wElbow, wJDist, wLim, outMargin, ikRotWeight, ikOriScoreWeight)
ik = inverseKinematics('RigidBodyTree', robot);
Tgoal = [Rgoal, pGoal(:); 0 0 0 1];
weights = [1 1 1 ikRotWeight ikRotWeight ikRotWeight];

seedList = { ...
    struct('q',qSeed,'type','qRef'), ...
    struct('q',qRef,'type','qHome'), ...
    struct('q',clampToLimits(applyFixedPerturbation(qSeed, 0.06), qMin, qMax),'type','perturb'), ...
    struct('q',clampToLimits(makeOutwardSeed(qRef, sideSign), qMin, qMax),'type','outward')};

bestScore = inf;
report = struct('ok',false,'posErr',inf,'elbowOutMetric',nan,'elbowWorldX',nan,'baseWorldX',baseWorldXYZ(1), ...
    'elbowDeltaX',nan,'oriErr',inf,'clampUsed',false,'exitFlag',-999,'score',inf,'q',qRef,'seedType','none','seedTypesTried','','jointLimitPenalty',inf);
tried = strings(1, numel(seedList));

for i = 1:numel(seedList)
    q0 = clampToLimits(seedList{i}.q, qMin, qMax);
    tried(i) = string(seedList{i}.type);
    [qCand, info] = ik(eeName, Tgoal, weights, q0);
    exitFlag = parseExitFlag(info);

    wasClamped = false;
    qCand2 = clampToLimits(qCand, qMin, qMax);
    if norm(qCand2 - qCand) > 1e-9
        wasClamped = true;
    end

    Tcur = getTransform(robot, qCand2, eeName);
    posErr = norm(Tcur(1:3,4).' - pGoal, 2);
    Rcur = Tcur(1:3,1:3);
    dR = Rgoal' * Rcur;
    c = (trace(dR)-1)/2;
    c = max(-1,min(1,c));
    oriErr = acos(c);
    [elbowOut, elbowWorldX, baseWorldX, deltaX] = elbowOutScalarWorld(robot, qCand2, shoulderBody, elbowBody, sideSign, baseWorldXYZ);
    limPenalty = jointLimitPenalty(qCand2, qMin, qMax);

    inwardPenalty = max(0, outMargin - elbowOut)^2;
    score = wPos*posErr + (4.0*wElbow)*inwardPenalty + wJDist*norm(qCand2 - qRef)^2 + wLim*limPenalty + ikOriScoreWeight*oriErr;

    if score < bestScore
        bestScore = score;
        report.ok = isfinite(posErr) && posErr < 0.08;
        report.posErr = posErr;
        report.elbowOutMetric = elbowOut;
        report.clampUsed = wasClamped;
        report.exitFlag = exitFlag;
        report.score = score;
        report.q = qCand2;
        report.seedType = seedList{i}.type;
        report.jointLimitPenalty = limPenalty;
        report.elbowWorldX = elbowWorldX;
        report.baseWorldX = baseWorldX;
        report.elbowDeltaX = deltaX;
        report.oriErr = oriErr;
    end
end
report.seedTypesTried = char(strjoin(tried, ','));
end

function qBias = applyAltBias(q)
qBias = q;
if numel(qBias) >= 2
    qBias(2) = qBias(2) + deg2rad(12);
end
if numel(qBias) >= 3
    qBias(3) = qBias(3) - deg2rad(10);
end
end

function qPert = applyFixedPerturbation(q, mag)
qPert = q;
for k = 1:numel(qPert)
    s = sin(0.7*k);
    qPert(k) = qPert(k) + mag*s;
end
end

function qOut = makeOutwardSeed(qBase, sideSign)
qOut = qBase;
if numel(qOut) >= 2
    qOut(2) = qOut(2) - sideSign*deg2rad(10);
end
if numel(qOut) >= 3
    qOut(3) = qOut(3) + sideSign*deg2rad(6);
end
if numel(qOut) >= 4
    qOut(4) = qOut(4) + sideSign*deg2rad(6);
end
end

function [eOut, elbowWorldX, baseWorldX, deltaX] = elbowOutScalarWorld(robot, q, shoulderBody, elbowBody, sideSign, baseWorldXYZ)
if isempty(shoulderBody) || isempty(elbowBody)
    eOut = 0;
    elbowWorldX = NaN;
    baseWorldX = baseWorldXYZ(1);
    deltaX = NaN;
    return;
end
TelW = getTransform(robot, q, elbowBody);
elbowWorldX = TelW(1,4);
baseWorldX = baseWorldXYZ(1);
deltaX = elbowWorldX - baseWorldX;
eOut = sideSign * deltaX;
end

function R = rotmZYX(yaw, pitch, roll)
cy = cos(yaw); sy = sin(yaw);
cp = cos(pitch); sp = sin(pitch);
cr = cos(roll); sr = sin(roll);
Rz = [cy -sy 0; sy cy 0; 0 0 1];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rx = [1 0 0; 0 cr -sr; 0 sr cr];
R = Rz * Ry * Rx;
end

function y = softplus(x, beta)
y = (1/beta)*log(1 + exp(beta*x));
end

function ef = parseExitFlag(info)
ef = -1;
if isstruct(info) && isfield(info,'ExitFlag')
    ef = info.ExitFlag;
end
end

function eOut = elbowOutScalar(robot, q, shoulderBody, elbowBody, sideSign)
if isempty(shoulderBody) || isempty(elbowBody)
    eOut = 0;
    return;
end
TelW = getTransform(robot, q, elbowBody);
Tbase = getBaseTransform(robot, q);
pElB = Tbase \ [TelW(1:3,4); 1];
elbowX = pElB(1);
eOut = sideSign * elbowX;
end

function Tbase = getBaseTransform(robot, q)
if any(strcmp(robot.BodyNames, 'world_base'))
    Tbase = getTransform(robot, q, 'world_base');
else
    Tbase = eye(4);
end
end

function penalty = jointLimitPenalty(q, qMin, qMax)
span = max(qMax-qMin, 1e-6);
u = (q-qMin)./span;
v = (qMax-q)./span;
penalty = mean(softplus(0.06-u,25) + softplus(0.06-v,25));
end

function q = clampToLimits(q, qMin, qMax)
q = min(max(q, qMin), qMax);
end

function qMin = extractJointMins(robot, qHome)
[qMin, ~] = extractJointLimits(robot, qHome);
end

function qMax = extractJointMaxs(robot, qHome)
[~, qMax] = extractJointLimits(robot, qHome);
end

function [qMin, qMax] = extractJointLimits(robot, qHome)
qMin = -inf(size(qHome));
qMax = +inf(size(qHome));

lims = [];
for i = 1:numel(robot.Bodies)
    j = robot.Bodies{i}.Joint;
    if strcmpi(j.Type,'fixed')
        continue;
    end
    pl = j.PositionLimits;
    if numel(pl) ~= 2 || any(~isfinite(pl))
        pl = [-pi, pi];
    end
    lims(end+1,:) = pl; %#ok<AGROW>
end

if size(lims,1) == numel(qHome)
    qMin = lims(:,1).';
    qMax = lims(:,2).';
else
    qMin(:) = -pi;
    qMax(:) = +pi;
end
end

function lims = computeStaticLimits(baseL, baseR, center, abc, goalL, goalR, capsL, capsR)
pts = [baseL; baseR; goalL; goalR; center + abc; center - abc];

for i = 1:numel(capsL)
    pts = [pts; capsL(i).p0; capsL(i).p1]; %#ok<AGROW>
end
for i = 1:numel(capsR)
    pts = [pts; capsR(i).p0; capsR(i).p1]; %#ok<AGROW>
end

mn = min(pts,[],1);
mx = max(pts,[],1);
pad = [0.20 0.25 0.20];
lims = [mn(1)-pad(1), mx(1)+pad(1); ...
        mn(2)-pad(2), mx(2)+pad(2); ...
        mn(3)-pad(3), mx(3)+pad(3)];
end


function p = projectInsideEllipsoid(p, c0, abc, scale)
a = abc(1); b = abc(2); c = abc(3);
X = (p(1)-c0(1))/a;
Y = (p(2)-c0(2))/b;
Z = (p(3)-c0(3))/c;
r2 = X^2 + Y^2 + Z^2;
if r2 > scale^2
    s = sqrt(r2)/scale;
    X = X/s; Y = Y/s; Z = Z/s;
    p = [c0(1)+a*X, c0(2)+b*Y, c0(3)+c*Z];
end
end

function ee = chooseEENameCommon(robotL, robotR, opts, eeAuto)
req = getOpt(opts, 'EEName', '');
if ~isempty(req) && any(strcmp(robotL.BodyNames, req)) && any(strcmp(robotR.BodyNames, req))
    ee = req;
    return;
end
if any(strcmp(robotL.BodyNames, 'link_6')) && any(strcmp(robotR.BodyNames, 'link_6'))
    ee = 'link_6';
    return;
end
if any(strcmp(robotL.BodyNames, eeAuto)) && any(strcmp(robotR.BodyNames, eeAuto))
    ee = eeAuto;
    return;
end
% safe fallback to last body name of left if shared, else left's last
cand = robotL.BodyNames{end};
if any(strcmp(robotR.BodyNames, cand))
    ee = cand;
else
    ee = robotL.BodyNames{end};
end
end

function [shoulderBody, elbowBody] = pickShoulderElbowBodies(robot)
names = string(robot.BodyNames);
shoulderCandidates = ["link2","link_2","shoulder","upperarm","upper_arm"];
elbowCandidates    = ["link3","link_3","elbow","forearm","lowerarm","lower_arm"];
shoulderBody = '';
elbowBody = '';

for c = shoulderCandidates
    idx = find(contains(lower(names), lower(c)), 1, 'first');
    if ~isempty(idx)
        shoulderBody = char(names(idx));
        break;
    end
end
for c = elbowCandidates
    idx = find(contains(lower(names), lower(c)), 1, 'first');
    if ~isempty(idx)
        elbowBody = char(names(idx));
        break;
    end
end

if isempty(shoulderBody)
    shoulderBody = char(names(min(2,numel(names))));
end
if isempty(elbowBody)
    elbowBody = char(names(min(3,numel(names))));
end
end

function [robotL, robotR] = placeDualRobotsYawOnly(robotRaw, baseL, baseR, yawDegL, yawDegR)
% Yaw-only base transform (roll/pitch forbidden)
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

function drawCapsuleSwept(ax, p0, p1, r, rgb, faceAlpha)
v = p1 - p0;
L = norm(v);
if L < 1e-8
    return;
end

[xc,yc,zc] = cylinder(r, 14);
zc = zc * L;

% Build orthonormal basis where ez aligns with segment direction
ez = v / L;
tmp = [0 0 1];
if abs(dot(ez,tmp)) > 0.95
    tmp = [0 1 0];
end
ex = cross(tmp, ez); ex = ex / norm(ex);
ey = cross(ez, ex);
R = [ex(:), ey(:), ez(:)];

P = R * [xc(:)'; yc(:)'; zc(:)'];
X = reshape(P(1,:), size(xc)) + p0(1);
Y = reshape(P(2,:), size(yc)) + p0(2);
Z = reshape(P(3,:), size(zc)) + p0(3);
surf(ax, X, Y, Z, 'FaceColor', rgb, 'FaceAlpha', faceAlpha, 'EdgeAlpha', 0.06, 'EdgeColor', rgb);

[xs,ys,zs] = sphere(10);
P1 = R * [r*xs(:)'; r*ys(:)'; r*zs(:)'];
X1 = reshape(P1(1,:), size(xs)) + p0(1);
Y1 = reshape(P1(2,:), size(ys)) + p0(2);
Z1 = reshape(P1(3,:), size(zs)) + p0(3);
surf(ax, X1, Y1, Z1, 'FaceColor', rgb, 'FaceAlpha', faceAlpha, 'EdgeAlpha', 0.03, 'EdgeColor', rgb);

X2 = reshape(P1(1,:), size(xs)) + p1(1);
Y2 = reshape(P1(2,:), size(ys)) + p1(2);
Z2 = reshape(P1(3,:), size(zs)) + p1(3);
surf(ax, X2, Y2, Z2, 'FaceColor', rgb, 'FaceAlpha', faceAlpha, 'EdgeAlpha', 0.03, 'EdgeColor', rgb);
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

function v = getOpt(opts, key, defaultVal)
if isfield(opts, key)
    v = opts.(key);
else
    v = defaultVal;
end
end
