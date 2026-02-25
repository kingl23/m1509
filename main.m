function main()
% MAIN - Doosan M1509 demo runner
% Folder layout expected:
%   <folder>/
%     main.m
%     workspace.m
%     test_trajectory.m
%     m1509.urdf
%     m1509/ (MF1509_*.dae)

clc; close all;

% workspace(20000);
% test_trajectory();
% dual_target_workspace(); % legacy
% New simple bootstrap-first demo example:
out = dual_arm_workspace_demo('Seed',1,'Mode','bootstrap','Visualize',true);
disp(out.success);



end