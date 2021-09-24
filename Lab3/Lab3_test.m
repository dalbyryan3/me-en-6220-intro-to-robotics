%% ME EN 6220 Lab 3 Test Ryan Dalby
%%
clear;
close all;
%% Test 1
disp(" ");
disp("Begin test 1");
a = [69 370.82]; % mm
d = [270.35 374.29 368.3]; % mm

% Test pose for placing above object
T_w_tool_test = [0.984 0.177 0.024 727.089;...
                0.177 -0.984 0.020 418.581;...
                0.027 -0.015 -0.999 732.233;...
                0 0 0 1];
disp("Running ikinelbow for placing above object");
theta_vals = ikinelbow(a,d,T_w_tool_test,1,1,1);

disp("Angles from inverse kinematics");
theta_fwd = [theta_vals 0];
disp(theta_fwd);

a_fwd = [a(1) a(2) 0 0 0 0 0];
d_fwd = [d(1) 0 0 d(2) 0 0 d(3)];
alpha_fwd = [-pi/2 0 -pi/2 pi/2 -pi/2 0 0];
disp("Running fkine with ikinelbow theta values");
T_0_tool = fkine(a_fwd,d_fwd,alpha_fwd,theta_fwd);
L = 221; % mm
h = 22; % mm
H = 1104; % mm
T_w_0 = [sqrt(2)/2 sqrt(2)/2 0 L;... 
         -sqrt(2)/2 sqrt(2)/2 0 h;...
         0 0 1 H;...
         0 0 0 1];
     
T_w_tool = T_w_0*T_0_tool;

disp("Pose from inverse kinematics");
disp(T_w_tool);

disp("True pose");
disp(T_w_tool_test);

disp("End test 1");
disp(" ");

%% Test 2
disp(" ");
disp("Begin test 2");
a = [69 370.82]; % mm
d = [270.35 374.29 368.3]; % mm

mat_file_to_load = "test_cases.mat";
load(mat_file_to_load);
solution_file_to_load = "test_cases_output.txt";
solution_file = fopen(solution_file_to_load);

T1_angles = ikinelbow(a,d,T1,1,1,1);
T2_angles = ikinelbow(a,d,T2,1,1,1);

disp("Case 1 inverse kinematics from ikinelbow");
disp(T1_angles);
disp("Case 1 inverse kinematics actual");
disp(fgetl(solution_file)); % Read a line
disp(" ");
disp(" ");

disp("Case 2 inverse kinematics from ikinelbow");
disp(T2_angles);
disp("Case 2 inverse kinematics actual");
disp(fgetl(solution_file)); % Read a line
disp(" ");
disp(" ");

disp("End test 2");
disp(" ");

