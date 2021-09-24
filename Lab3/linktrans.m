% linktrans returns homogeneous transformation matrix for a link
%  
%	T = linktrans(a,d,alpha,theta) Converts from standard DH parameters
%   to the corresponding homogeneous transformation matrix
%
%	a     = the a_i DH parameter associated with link
%	d     = the d_i DH parameter associated with link
%	alpha = the alpha_i DH parameter associated with link
%   theta = the theta_i DH parameter associated with link
%   T     = 4x4 homogenous transformation matrix representing the link
%  
%	Ryan Dalby
%	ME EN 6220
%	10/23/2020

function [T] = linktrans(a,d,alpha,theta)

T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
    0, sin(alpha), cos(alpha), d;...
    0, 0, 0, 1];
end

