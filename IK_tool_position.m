%% %% ME384R - ASBR - THA4
% Written by Clara Summerford and Nathan Lovell
%
% Finds the inverse kinematics iteratively using the body Jacobian of the
% robot and knowing a desired final end-effector position. Uses the 
% Newton-Raphson root-finding method as a basis for finding a
% solution that minimizes the error to near zero. The error is the
% difference between the desired end-effector position and current
% end-effector positon of each iteration. 

% Input:
% robot = a struct of the following form:
% % % panda.screws = 6xn space Jacobian in terms of symbolic thetas
% % % panda.transf.T1 = Joint 1 transformation matrix (zero position)
% % % panda.transf.Tn = Joint n transformation matrix (zero position)
% % % panda.transf.M = End-effector transformation matrix (zero position)
% % % panda.transf.Ts = Space frame transformation matrix (typically I_4x4)
% % % panda.transf.Tbs = Space frame transformation matrix w.r.t
% % % end-effector frame
% % % panda.limits = nx2 matrix of joint limits in the form [lower,upper]
% config_a = 1xn vector of joint angles corresponding to the current robot
% position
% config_b = 4x4 transformation matrix corresponding to the desired end
% effector position in the spacial frame 
% plot = Boolean value to allow the test function to plot all IK results by
% storing all joint values as algorithm converges, not just final result.
%
% Output: 
% t_vec = 1xn vector of joint angles corresponding to the joint-space 
% orientation of the robot in config b (desired EE position)
% t_vec_mat = mxn array of joint angles for each iteration until the IK
% algorithm converges, used to plot during function testing and validation


function [t_vec, t_vec_mat] = IK_tool_position(robot, config_a, config_b, plot)

    Tsd = config_b;
    p_goal = Tsd(1:3,4);
    M = robot.transf.M;
    %t_new = zeros(7,1);

    t_vec = config_a';
    fprintf("Start config (t_vec):\n")
    disp(t_vec)

    physical_err = 10; % initializing arbitrarily high error to start loop

    it = 0; % initializing iteration counter

    % initializing array to append joint angles if plotting is desired
    if plot == true
        t_vec_mat = config_a;
    end

    % loop until desired error thresholds are met
    % accounting for 3mm constraint in loop
    while physical_err > 0.003
        
        % position at current joint guess
        Tsb = FK_space(robot,t_vec',0);


        % print out results from iteration
        fprintf("Iteration: %d\n", it)

        % re-calculating the space Jacobian for each iteration
        Js = J_space(robot, t_vec');

        % Formulate least-squares optimization

        % Objective function
        %t = M*p_tip 
        t = Tsb(1:3,4); % Have to figure out how I'm dealing with the tip
        %t = [0;0;0];
        J_a = Js(1:3,:);
        J_e = Js(4:6,:);
        C = vec2SkewSym(-t)*J_a + J_e;
        d = -t + p_goal;

        % Constraints
        lb = robot.limits(:,1) - t_vec; % Lower joint limits
        ub = robot.limits(:,2) - t_vec; % Upper joint limits

        % Solve least-squares optimization
        x0 = [0 0 0 0 0 0 0]';
        dt_vec = lsqlin(C,d,[],[],[],[],lb,ub,x0);

        est_err = norm(C*dt_vec - d)

        % updating values for next iteration
        t_new = t_vec + dt_vec;
        %t_new = t_new + dt_vec
        t_vec = t_new;
        fprintf("t_vec: \n")
        disp(t_vec)
        it = it + 1; % incrementing iteration counter

        T_est = FK_space(robot, t_new', false);
        p_est = T_est(1:3, 4);
        physical_err = norm(p_goal-p_est)

        if plot == true
            t_vec_mat = [t_vec_mat; t_vec'];
        else
            t_vec_mat = 0;
        end

    end

    % transposing nx1 t_vec array back to 1xn array used for other functions
    t_vec = t_vec'; 

end 