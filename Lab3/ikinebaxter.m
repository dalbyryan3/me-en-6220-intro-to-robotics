% ikinebaxter computes the inverse kinematics of a specified robot
%  
%	theta = ikinebaxter(a1,a2,d1,d4,d6,T,LR,UD,NF) Calculates a vector of
%	theta (joint angle) values given specification of the robot geometery
%	and a desired end position
%
%	a1 = DH parameter a1 value for specified robot
%   a2 = DH parameter a2 value for specified robot 
%   dl = DH parameter d1 value for specified robot
%   d4 = DH parameter d4 value for specified robot
%   d6 = DH parameter d6 value for specified robot
%   T  = Homogeneous transformation matrix which specifies a desired
%   location of the endpoint from 6 with respect to frame 0
%   LR = Choose 1 for "lefty" solution and 0 for "righty" solution
%   UD = Choose 1 for "elbow up" solution and 0 for "elbow down" solution 
%   NF = Choose 1 for for flip which specifies joint angle 5 is in quadrants 2 or 3
%   and choose 0 for no flip which specifies joint angle 5 is in quadrants 2 or 3 
%  
%	Ryan Dalby
%	ME EN 6220
%	10/23/2020
function [theta] = ikinebaxter(a1,a2,d1,d4,d6,T,LR,UD,NF)
theta = [];
R_0_6 = T(1:3,1:3);
d_0_06 = T(1:3,4);

%
% Workspace determination
%
d_0_06_mag = norm(d_0_06);
inner_primary_workspace_diameter = a1-a2-d4+d6;
if(inner_primary_workspace_diameter < 0)
    inner_primary_workspace_diameter = 0;
end
outer_primary_workspace_diameter = a1+a2+d4-d6;
if(outer_primary_workspace_diameter < 0)
    outer_primary_workspace_diameter = 0;
end
% Make sure we are in primary workspace or exit function
if (d_0_06_mag < inner_primary_workspace_diameter) || (d_0_06_mag > outer_primary_workspace_diameter)
    disp('Outside of primary workspace');
    return;
end


%  
% Regional structure inverse kinematics
%
d_6_46 = [0; 0; d6];
d_0_46 = R_0_6 * d_6_46;
d_0_04 = d_0_06 - d_0_46;

% theta 1 determination 
theta(1) = atan2(d_0_04(2), d_0_04(1));
if (~LR)
    theta(1) = theta(1) + pi;
end

% theta 3 and theta 2 determination 
d_0_01 = [a1*cos(theta(1)); a1*sin(theta(1)); d1];
R_1_0 = [cos(theta(1)) sin(theta(1)) 0;... 
         0 0 -1;...
         -sin(theta(1)) cos(theta(1)) 0];
d_1_14 = R_1_0 * (d_0_04-d_0_01);

r2 = d_1_14(1)^2 + d_1_14(2)^2;

alpha = 2*atan2(sqrt((a2+d4)^2 - (r2)) , sqrt((r2) - (a2-d4)^2));  
if(~UD)
    alpha = alpha * -1;
end
theta(3) = alpha - pi/2;

psi = atan2((d4*sin(alpha)),(a2+d4*cos(alpha)));
phi = atan2(d_1_14(2),d_1_14(1));
theta(2) = phi - psi;


%
% Orientation structure inverse kinematics
%
R_0_1 = transpose(R_1_0);
R_1_2 = [cos(theta(2)) -sin(theta(2)) 0;... 
         sin(theta(2)) cos(theta(2)) 0;...
         0 0 1];
R_2_3 = [cos(theta(3)) 0 -sin(theta(3));... 
         sin(theta(3)) 0 cos(theta(3));...
         0 -1 0];
R_0_3 = R_0_1*R_1_2*R_2_3;

R_3_6 = transpose(R_0_3) * R_0_6;

% theta 4 determination
theta(4) = atan2(-R_3_6(2,3),-R_3_6(1,3));

% theta 5 and theta 6 determination
R_3_4 = [cos(theta(4)) 0 sin(theta(4));... 
         sin(theta(4)) 0 -cos(theta(4));...
         0 1 0];

R_4_6 = transpose(R_3_4) * R_3_6;

theta(5) = atan2(-R_4_6(1,3),R_4_6(2,3));
if(NF)
    theta(5) = theta(5) + pi;
end

theta(6) = atan2(-R_4_6(3,1),-R_4_6(3,2));
    
end

