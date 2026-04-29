%% %% ME384R - ASBR - THA4 Presentation
% Written by Clara Summerford and Nathan Lovell
%
% Takes the t_vec_mat produced by the IK functions and creates a gif of the
% robot (plotted using the MATLAB Robotics Toolbox) converging on its 
% final configuration. Includes a predefined path in the gif.
%
% Inputs:
% t_vec_mat = mxn array of joint angles for each iteration until the IK
% algorithm converges, used to plot during function testing and validation
% filename = desired file name (MUST include the .gif extension!)

function IK_MRB_gif_w_path(robot,t_vec_mat,filename)
    
    %%% Define a series of waypoints along an arbitrary path. As an example, the
    %%% below waypoints trace out an L shape.
    start = FK_space(robot,t_vec_mat(1,:),0);

    % Initialize
    wp{1} = start;
    
    % Create waypoints of the vertical part of the L
    dz = [0 0 0 0; 0 0 0 0; 0 0 0 -0.05; 0 0 0 0]; % Subtract 50mm in negative z direction
    for i = 1:5
        waypoint =  wp{i} + dz;
        wp{i+1} = waypoint;
    end
    
    % Create waypoints of the horizontal part of the L
    dx = [0 0 0 0.05; 0 0 0 0; 0 0 0 0; 0 0 0 0]; % Add 50mm in positive x direction
    for i = 6:10
        waypoint =  wp{i} + dx;
        wp{i+1} = waypoint;
    end
    
    % Build and plot the L shape
    for u = 1:length(wp)
        L(u,:) = wp{u}(1:3,4)';
    end

    % Create gif using FK plots
    fig1 = figure(1);
    
    % Plot each configuration iteratively
    for i = 1:size(t_vec_mat,1)
        % Load franka from MATLAB Robotics Toolbox
        ml_robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
    
        % Get configuration of MATLAB robot described by t_vec_mat(i)
        base = homeConfiguration(ml_robot);
        config = zeros(length(base),1);
        config(1:length(t_vec_mat(i,:))) = t_vec_mat(i,:)';
    
        cla(fig1,"reset")
        show(ml_robot,config);
        view(-37.5,30); % View x-z plane
        hold on
        plot3(L(:,1),L(:,2),L(:,3),'-m','LineWidth',1.5)

        % Create gif
        [image, map] = rgb2ind(frame2im(getframe(fig1)),256);
  
        if i == 1
            % For the first frame, create the file with infinite loop count
            imwrite(image, map, filename, 'LoopCount', Inf, 'DelayTime', 0.2);
        else
            % For subsequent frames, append to the existing file
            imwrite(image, map, filename,'gif','WriteMode','append', 'DelayTime', 0.2);
        end

    end

end