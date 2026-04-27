close all

test1 = [0 pi/2 0 pi/4 0 pi/6 0]; % This example is outside of the joint limits...
test2 = [0 pi/2 0 pi/4 0 pi/7 0];
test3 = [0 0 0 -pi/2 0 0 0]; % Doesn't converge for this one??
test4 = [0 0 0 -pi/4 0 0 0];
test5 = [0 pi/2 0 -pi/4 0 pi/6 0];
test6 = [0 pi/2 0 -pi/4 0 pi/7 0];
test7 = [0 -pi/6 pi/4 -pi/6 0 pi/2 0];
test8 = [0 -pi/6 pi/4 -pi/7 0 pi/2 0];

% plot3(0, 0, 0);
% view(3)
T1 = FK_space(panda, test3, true);
pause(1)
T2 = FK_space(panda, test4, true);
pause(1)
% view(3)
% figure

[test_result, t_vec_mat] = IK_tool_position(panda, test3, T2, true)

% Visualize the iterations
IK_gif(panda,t_vec_mat,'IK_tool_position_test1.gif')

% figure
test_plot = FK_space(panda, test_result, true);

% Plot first iteration of the optimization
test_plot2 = FK_space(panda, t_vec_mat(2,:), true)
