%% %% ME384R - ASBR - THA4
% Written by Clara Summerford and Nathan Lovell
%
% Tests the output of IK_tool_orientation.m to ensure that it passes the
% constraint of staying 3mm from the goal point. Also returns four
% performance measures for analysis, and plots the starting, target, and 
% end configurations of the robot.
%
% Inputs:
% robot = a struct of the following form:
% % % panda.screws = 6xn space Jacobian in terms of symbolic thetas
% % % panda.transf.T1 = Joint 1 transformation matrix (zero position)
% % % panda.transf.Tn = Joint n transformation matrix (zero position)
% % % panda.transf.M = End-effector transformation matrix (zero position)
% % % panda.transf.Ts = Space frame transformation matrix (typically I_4x4)
% % % panda.transf.Tbs = Space frame transformation matrix w.r.t
% % % end-effector frame
% % % panda.limits = nx2 matrix of joint limits in the form [lower,upper]
% test1 = 1x7 vector of joint angles describing the start configuration
% test2 = 1x7 vector of joint angles describing the desired end 
% configuration (really just used to extract the target point, p_goal)
%
% Outputs:
% test = a boolean; true if the final position of the end effector is
% within 3 mm of the target point
% numit = number of iterations for convergence on final configuration
% e_f = final translation error (distance between the final end-effector 
% position and the goal point, in m
% e_avg = average translation error (average distance between the configuration 
% at each iteration and the goal point), in m
% theta_avg = average change in rotation, in rad, between successive
% configurations as the solver iterates
%
% Example tests:
% % Test a small transformation:
% test1 = [0 0 0 -pi/2 0 0 0];
% test2 = [0 0 0 -pi/2.1 0 0 0];
% 
% % Test a bigger transformation:
% test3 = [0 0 0 -pi/2 0 0 0];
% test4 = [0 0 0 -pi/4 0 0 0];
% 
% % Test a transformation that nearly violates joint limits:
% test5 = [0 0 0 -pi/12 0 0 0];
% test6 = [0 0 0 pi/12 0 0 0];

function [test, N, e_f, e_avg, theta_avg] = IK_tool_orientation_test(robot, test1, test2)

    % Plot 
    T1 = FK_space(robot, test1, true);
    pause(1)
    T2 = FK_space(robot, test2, true);
    pause(1)
    
    % Solve for the transformations to reach the goal
    [test_result, t_vec_mat] = IK_tool_orientation(robot, test1, T2, true);
    
    % Plot and verify the final position is within 3mm of the goal
    T_f = FK_space(robot, test_result, 1);

    p_goal = T2(1:3,4);
    p_f = T_f(1:3, 4);
    e_f = norm(p_goal-p_f); % final translation error

    if e_f < 0.003
        test = true;
        disp('Test PASSED')
    else
        test = false;
        disp('Test FAILED')
    end

    % Calculate number of iterations
    N = size(t_vec_mat,1) - 1;

    % Calculate the translation errors and rotation changes
    for i = 1:(size(t_vec_mat,1) - 1)
        % Configuration of the current iteration
        T_it = FK_space(robot,t_vec_mat(i+1,:),0);

        % Position and orientation of the current iteration
        R_it = T_it(1:3,1:3);
        p_it = T_it(1:3,4);

        % Calculate the translation error for the current iteration
        e_it(i) = norm(p_goal-p_it);

        % Calculate the angle of rotation betweent the current and previous
        % configurations
        T_prev = FK_space(robot,t_vec_mat(i,:),0);
        R_prev = T_prev(1:3,1:3);
        dR = R_it*R_prev';
        theta(i) = acos((trace(dR)-1)/2);

    end

    % Calculate average translation error across the iteration
    e_avg = sum(e_it)/length(e_it);

    % Calculate the average change in rotation between iterations
    theta_avg = sum(theta)/length(theta);

    % % Visualize the iterations
    % figure
    % IK_gif(robot,t_vec_mat,'IK_tool_orientation_test1.gif')
    % figure
    % IK_MRB_gif(t_vec_mat,'IK_tool_orientation_mltest1.gif')

end