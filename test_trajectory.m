function test_trajectory()
% TEST_TRAJECTORY - Create an arbitrary joint-space trajectory and visualize motion.
% Produces an animation of the robot moving along a generated trajectory.

%% ---- Resolve base folder ----
baseDir = fileparts(mfilename("fullpath"));
cd(baseDir);
addpath(genpath(baseDir));

%% ---- Load robot (URDF patch + auto EE) ----
[robot, eeName, qHome, jointNames] = loadRobotAutoEE(baseDir);
fprintf("[test_trajectory] EE used: %s\n", eeName);

%% ---- Joint limits ----
nJ = numel(qHome);
qMin = zeros(1,nJ);
qMax = zeros(1,nJ);

for i = 1:nJ
    jName = jointNames(i);
    ownerBody = [];
    for b = 1:numel(robot.Bodies)
        if strcmp(robot.Bodies{b}.Joint.Name, jName)
            ownerBody = robot.Bodies{b};
            break;
        end
    end
    assert(~isempty(ownerBody), "Could not find joint owner body for joint: %s", jName);

    lim = ownerBody.Joint.PositionLimits;
    qMin(i) = lim(1);
    qMax(i) = lim(2);
end

isInf = isinf(qMin) | isinf(qMax);
if any(isInf)
    warning("Some joints have infinite limits. Clamping those to [-pi, pi].");
    qMin(isInf) = -pi;
    qMax(isInf) =  pi;
end

%% ---- Build an arbitrary trajectory in joint space ----
% Strategy:
%  - Choose a few random waypoints within joint limits
%  - Interpolate smoothly (pchip) per joint
%  - Animate at ~30 FPS

rng(7);

numWaypoints = 6;
Ttotal = 6.0;        % seconds
fps = 30;
N = max(2, round(Ttotal * fps));

tWp = linspace(0, Ttotal, numWaypoints);
t   = linspace(0, Ttotal, N);

% Waypoints: start at home, then random, end near home
W = zeros(numWaypoints, nJ);
W(1,:) = qHome;

% Random waypoints (limit 중앙부 위주로)
center = 0.5*(qMin+qMax);
span   = 0.35*(qMax-qMin);   % 너무 끝까지 가지 않게
for k = 2:numWaypoints-1
    W(k,:) = center + (2*rand(1,nJ)-1).*span;
end
W(end,:) = qHome + 0.2*(2*rand(1,nJ)-1).*span;  % 마지막은 home 근처

% Interpolate each joint with pchip (overshoot 적음)
Q = zeros(N, nJ);
for j = 1:nJ
    Q(:,j) = interp1(tWp, W(:,j), t, "pchip");
end

% Safety clamp
Q = max(min(Q, qMax), qMin);

%% ---- Prepare figure ----
figure("Color","w");
ax = axes; hold(ax, "on");

% Draw once to get axis limits; keep camera fixed during animation
h = show(robot, qHome, "Parent", ax, "PreservePlot", false, "Visuals","on", "Frames","off");
axis(ax, "equal"); grid(ax, "on");
xlabel(ax, "X [m]"); ylabel(ax, "Y [m]"); zlabel(ax, "Z [m]");
title(ax, "M1509 Test Trajectory (Joint-space) - Animation");
view(ax, 35, 20);

% Fix axis limits to reduce flicker
xlim0 = xlim(ax); ylim0 = ylim(ax); zlim0 = zlim(ax);

% Optional: trace EE path
eePath = zeros(N,3);
pltEE = plot3(ax, nan, nan, nan, "-", "LineWidth", 1);

rc = rateControl(fps);

%% ---- Animate ----
for k = 1:N
    q = Q(k,:);
    show(robot, q, "Parent", ax, "PreservePlot", false, "Visuals","on", "Frames","off");

    % Maintain axis bounds
    xlim(ax, xlim0); ylim(ax, ylim0); zlim(ax, zlim0);

    % Update EE trace
    T = getTransform(robot, q, eeName);
    eePath(k,:) = tform2trvec(T);
    set(pltEE, "XData", eePath(1:k,1), "YData", eePath(1:k,2), "ZData", eePath(1:k,3));

    drawnow;
    waitfor(rc);
end

hold(ax, "off");
end

%% ================= Local helper =================
function [robot, eeName, qHome, jointNames] = loadRobotAutoEE(baseDir)
urdfPath = fullfile(baseDir, "m1509.urdf");
assert(isfile(urdfPath), "URDF not found: %s", urdfPath);

% Patch package:// -> m1509/ if needed
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
cfgS = homeConfiguration(robot);
jointNames = string({cfgS.JointName});
qHome = [cfgS.JointPosition];

robot.DataFormat = "row";

% Auto EE as last leaf
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