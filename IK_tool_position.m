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

    % error thresholds
    w_err = 0.001;
    v_err = 0.001;

    Vb_v_err = 1000; % initializing arbitrarily high estimate to start loop
    Vb_w_err = 1000; 
    it = 0; % initializing iteration counter

    % initializing array to append joint angles if plotting is desired
    if plot == true
        t_vec_mat = config_a;
    end

    % loop until desired error thresholds are met
    while (Vb_v_err > v_err) || (Vb_w_err > w_err)
        
        % position at current joint guess
        Tsb = FK_space(robot,t_vec',0);

        % calculate error of guess
        Vb_skew = logm(inv(Tsb)*Tsd);
        Vb_w = [Vb_skew(3,2), Vb_skew(1,3), Vb_skew(2,1)]'; 
        Vb_v = Vb_skew(1:3, 4);
        Vb = [Vb_w; Vb_v];

        % error calculation will use maximum of error from magnitude of 
        % angular or linear velocities
        Vb_w_err = norm(Vb_w);
        Vb_v_err = norm(Vb_v);

        % print out results from iteration
        fprintf("Iteration: %d\n", it)
        fprintf("Vb w error: %d\n", Vb_w_err)
        fprintf("Vb v error: %d\n", Vb_v_err)

        % re-calculating the space Jacobian for each iteration
        Js = J_space(robot, t_vec');

        % % check if Jacobian is square
        % if size(Jb, 1) == size(Jb, 2) 
        %     % use normal jacobian
        %     j_dag = inv(Jb);
        % 
        % % redundant case, use right matrix pseudoinverse
        % elseif size(Jb,1) < size(Jb,2) 
        %     j_dag = Jb'*inv(Jb*Jb');
        % 
        % % underactuated case, use left pseudoinverse 
        % else 
        %     j_dag = (inv(Jb'*Jb))*Jb';
        % end

        %%% TEST: Formulate least-squares optimization

        % Objective function
        %t = M*p_tip 
        t = Tsb(1:3,4) % Have to figure out how I'm dealing with the tip
        %t = [0;0;0];
        J_a = Js(1:3,:);
        J_e = Js(4:6,:);
        C = vec2SkewSym(-t)*J_a + J_e;
        d = -(t + p_goal);
        %d = p_goal; % Do I have to change func to only accept p_goal or can I extract from config_b
        %d = 

        % Constraints
        lb = robot.limits(:,1) - t_vec; % Lower joint limits
        ub = robot.limits(:,2) - t_vec; % Upper joint limits
        % Not including 3mm constraint (yet) because redundant and
        % nonlinear

        % Solve least-squares optimization
        x0 = [0 0 0 0 0 0 0]';
        dt_vec = lsqlin(C,d,[],[],[],[],lb,ub,x0)

        norm(C*dt_vec - d)

        % Need an if clause somewhere that checks the 3mm constraint


        % updating values for next iteration
        t_new = t_vec + dt_vec;
        %t_new = t_new + dt_vec
        t_vec = t_new;
        fprintf("t_vec: \n")
        disp(t_vec)
        it = it + 1; % incrementing iteration counter

        if plot == true
            t_vec_mat = [t_vec_mat; t_vec'];
        else
            t_vec_mat = 0;
        end

    end

    % transposing nx1 t_vec array back to 1xn array used for other functions
    t_vec = t_vec'; 

end 