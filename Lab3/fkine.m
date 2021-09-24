% fkine returns homogeneous transformation matrix for the forward 
% kinematics of a robot
%  
%	T = fkine(a,d,alpha,theta) Calculates a homogenous transformation
%	matrix which represents the forward kinematics of a robot given a
%	sequential vector of its DH parameters
%
%	a     = a sequential vector of the a_i DH parameter associated with robot
%	d     = a sequential vector of the d_i DH parameter associated with robot
%	alpha = a sequential vector of the alpha_i DH parameter associated with robot
%   theta = a sequential vector of the theta_i DH parameter associated with robot
%   T     = 4x4 homogenous transformation matrix reprenenting the forward
%   kinematics of the robot
%  
%	Ryan Dalby
%	ME EN 6220
%	10/23/2020

function [T] = fkine(a,d,alpha,theta)
T = eye(4);
for i = 1:length(a)
    T = T * linktrans(a(i),d(i),alpha(i),theta(i));
end

