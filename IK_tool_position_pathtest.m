close all

% Define a starting position
t_vec = [0 0 0 -pi/2 0 0 0];
start = FK_space(panda,t_vec,1);

% Define a series of waypoints along an arbitrary path. As an example, the
% below waypoints trace out an L shape.

% Initialize
wp{1} = start;
dz = [0 0 0 0; 0 0 0 0; 0 0 0 -0.05; 0 0 0 0]; % Subtract 50mm in negative z direction

for i = 1:5
    waypoint =  wp{i} + dz;
    wp{i+1} = waypoint;
end

dx = [0 0 0 0.05; 0 0 0 0; 0 0 0 0; 0 0 0 0]; % Add 50mm in positive x direction

for i = 6:10
    waypoint =  wp{i} + dx;
    wp{i+1} = waypoint;
end


% Control robot path along the waypoints iteratively
% Initialize
wp_vec = []

for r = 1:(size(wp,2)-1)
    % Solve for the new joint angles
    [t_vec, t_vec_mat] = IK_tool_position(panda, t_vec, wp{r+1}, true)

    % Store each t_vec in a waypoint vector
    wp_vec = [wp_vec;t_vec];
end

% Plot the waypoint configurations one by one
for k = 1:length(wp_vec)
    cla
    config = FK_space(panda,wp_vec(k,:),1);
    pause(1)
end