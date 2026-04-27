close all

test1 = [0 0 0 -pi/2 0 0 0];
test2 = [0 0 0 -pi/3 0 0 0];


% plot3(0, 0, 0);
% view(3)
T1 = FK_space(panda, test1, true);
pause(1)
T2 = FK_space(panda, test2, true);
pause(1)
% view(3)
% figure

[test_result, t_vec_mat] = IK_tool_orientation(panda, test1, T2, true)

% Visualize the iterations
IK_gif(panda,t_vec_mat,'IK_tool_orientation_test2.gif')

% figure
test_plot = FK_space(panda, test_result, true);

% Plot first iteration of the optimization
test_plot2 = FK_space(panda, t_vec_mat(2,:), true)

