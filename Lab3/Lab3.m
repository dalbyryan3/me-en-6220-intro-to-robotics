%% ME EN 6220 Lab 3 Test Ryan Dalby
%%
clear;
close all;
%% Read poses from .mat file and produce inverse kinematics
a = [69 370.82]; % mm
d = [270.35 374.29 368.3]; % mm

mat_file_to_load = "poses.mat";
load(mat_file_to_load);
file_to_output = "joint_angles.txt";

% Will solve for lefty, elbow up, flip solution
theta_vals(1,:) = ikinelbow(a,d,above_object,1,1,1);
theta_vals(2,:) = ikinelbow(a,d,object,1,1,1);
theta_vals(3,:) = ikinelbow(a,d,midpoint,1,1,1);
theta_vals(4,:) = ikinelbow(a,d,above_goal,1,1,1);
theta_vals(5,:) = ikinelbow(a,d,goal,1,1,1);

fileID = fopen(file_to_output, "w");
theta_vals_size = size(theta_vals);
for i = 1:theta_vals_size(1)
    fprintf(fileID, "%f, %f, %f, %f, %f, %f\n", theta_vals(i,:));
end
fclose(fileID);
