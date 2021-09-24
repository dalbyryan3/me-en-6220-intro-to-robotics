%% ME EN 6220 Lab 1 Ryan Dalby
%%
clear;
close all;

%% Lab 1b
uppercut_start_pos = [-2.75771,0.879738,-0.203636,0.867466,-0.5184,-0.02646,3.04725];
uppercut_mid_pos = [-3.0323,1.3127,-0.859796,0.333257,0.234699,1.03275,3.04725];
uppercut_end_pos = [-2.75771,0.879738,-0.203636,0.867466,-0.518486,-0.0264612,3.04725];

jab_start_pos = [-1.94854,2.14566,0.233549,1.01856,0.552617,0.595568,3.04687];
jab_mid_pos = [-2.45552,0.576393,-0.466714,0.132306,2.71285,0.137291,2.94601];
jab_end_pos = [-1.94854,2.14566,0.233549,1.01856,0.552617,0.595568,3.04687];

t1 = 2.5; % s
t2 = 5.0; % s
t_step = .1; % s

% Calculate polynomial splice parameters
[~,uppercut_x] = TwoThirdDegreePolynomialSpliceSolve(t1, t2, t_step, uppercut_start_pos, uppercut_mid_pos, uppercut_end_pos, 1, 'Joint Angles versus Time for an Uppercut', 4);
[~,jab_x] = TwoThirdDegreePolynomialSpliceSolve(t1, t2, t_step, jab_start_pos, jab_mid_pos, jab_end_pos, 1, 'Joint Angles versus Time for a Jab', 4);

% Write to text file
dlmwrite('uppercut_trajectory.txt', t_step);
dlmwrite('uppercut_trajectory.txt', uppercut_x,'-append');
dlmwrite('jab_trajectory.txt', t_step);
dlmwrite('jab_trajectory.txt', jab_x,'-append');


function [t, x_vec] = TwoThirdDegreePolynomialSpliceSolve(t1, t2, t_step, start_pos, mid_pos, end_pos, should_plot, title_string, plot_width)
    if(should_plot)
        figure;
        sgtitle(title_string);
    end

    % Set vector sizes for coefficients to be solved for
    a2_vec = zeros(size(start_pos));
    a3_vec = zeros(size(start_pos));
    b1_vec = zeros(size(start_pos));
    b2_vec = zeros(size(start_pos));
    b3_vec = zeros(size(start_pos));


    % Set coefficients that are directly solved from constraints
    a0_vec = start_pos;
    b0_vec = mid_pos;
    a1_vec = zeros(size(start_pos));
    
    % Set t_values
    t_vec1 = 0:t_step:t1;
    t_vec2 = t1:t_step:t2;
    t = [t_vec1 t_vec2];
    x_vec = zeros(length(t),length(start_pos));
    
    % Declare 

    for i = 1:length(start_pos)
        syms a2 a3 b1 b2 b3
        eqn1 = start_pos(i) + a2*t1^2 + a3*t1^3 == mid_pos(i);
        eqn2 = mid_pos(i) + b1*(t2-t1) + b2*(t2-t1)^2 + b3*(t2-t1)^3 == end_pos(i);
        eqn3 = b1 + 2*b2*(t2-t1) + 3*b3*(t2-t1)^2 == 0;
        eqn4 = 2*a2*t1 + 3*a3*t1^2 == b1;
        eqn5 = 2*a2 + 6*a3*t1 == 2*b2;
        sol = solve([eqn1,eqn2,eqn3,eqn4,eqn5], [a2,a3,b1,b2,b3]);

        a2_vec(i) = sol.a2;
        a3_vec(i) = sol.a3;
        b1_vec(i) = sol.b1;
        b2_vec(i) = sol.b2;
        b3_vec(i) = sol.b3;

        % Fit the following polynomials to the indicated constraints 
        x1 = a0_vec(i) + a1_vec(i).*t_vec1 + a2_vec(i).*t_vec1.^2 + a3_vec(i).*t_vec1.^3;
        x2 = b0_vec(i) + b1_vec(i).*(t_vec2-t1) + b2_vec(i).*(t_vec2-t1).^2 + b3_vec(i).*(t_vec2-t1).^3;
        x = [x1 x2];
        x_vec(:,i) = x;

        if(should_plot)
            subplot(2,plot_width,i);
            plot(t,x);
            hold on;
            xlabel('Time (s)');
            ylabel(sprintf('Joint %d Angle', i));
        end
    end
end