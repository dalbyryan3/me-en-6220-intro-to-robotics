%% ME EN 6220 Homework 6 Ryan Dalby
%%
clear;
close all;
%%
% 3c
a = [69 370.82 0 0 0 0]
d = [270.35 0 0 374.29 0 368.3]
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0]
theta_fwd = [pi/2 -pi/4 pi/3 pi pi/6 pi/2]
disp("Running fkine");
T = fkine(a,d,alpha,theta_fwd)

% 5
disp("Running ikinebaxter");
theta_inv = ikinebaxter(a(1),a(2),d(1),d(4),d(6),T,1,1,0)
