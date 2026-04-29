%% %% ME384R - ASBR - THA4
% Written by Clara Summerford and Nathan Lovell
%
% Commands a robot to reach a target position with its end-effector while
% staying within the robot's joint limits. This function achieves this goal
% via a linear least-squares optimization instead of using inverse
% kinematics. The problem assumes the goal is to keep the end-effector
% within 3mm of the target point. Starting and target points are given by
% config_a and config_b, respectively.
% 
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

    Tsd = config_b; % Target configuration
    p_goal = Tsd(1:3,4); % Goal point - location of Tsd

    t_vec = config_a'; % Current joint angles
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

        % print out iteration number
        fprintf("Iteration: %d\n", it)

        % re-calculating the space Jacobian for each iteration
        Js = J_space(robot, t_vec');

        % Formulate least-squares objective function
        t = Tsb(1:3,4); % Tip location corresponds to end-effector frame after tool added
        J_a = Js(1:3,:); % Angular component of space Jacobian
        J_e = Js(4:6,:); % Linear component of space Jacobian
        C = vec2SkewSym(-t)*J_a + J_e;
        d = -t + p_goal;

        % Optimization constraints
        lb = robot.limits(:,1) - t_vec; % Lower joint limits
        ub = robot.limits(:,2) - t_vec; % Upper joint limits

        % Solve least-squares optimization
        x0 = zeros(7,1);
        options = optimoptions('lsqlin','Algorithm','active-set');
        [dq_vec,resnorm] = lsqlin(C,d,[],[],[],[],lb,ub,x0,options);

        % updating values for next iteration
        t_new = t_vec + dq_vec;
        t_vec = t_new;
        fprintf("t_vec: \n")
        disp(t_vec)
        it = it + 1; % incrementing iteration counter

        % New current position of end-effector (tool tip)
        T_est = FK_space(robot, t_new', false);

        % Calculate distance from goal (physical error)
        p_est = T_est(1:3, 4);
        physical_err = norm(p_goal-p_est);

        if plot == true
            t_vec_mat = [t_vec_mat; t_vec'];
        else
            t_vec_mat = 0;
        end

    end

    % transposing nx1 t_vec array back to 1xn array used for other functions
    t_vec = t_vec'; 

end 