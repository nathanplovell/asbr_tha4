test1 = [0 pi/2 0 pi/4 0 pi/6 0];
test2 = [0 pi/2 0 pi/4 0 pi/7 0];


% plot3(0, 0, 0);
% view(3)
T1 = FK_space(panda, test1, true);
pause(1)
T2 = FK_space(panda, test2, true);
pause(1)
% view(3)
% figure

test_result = IK_tool_position(panda, test1, T2, true)

% figure
test_plot = FK_space(panda, test_result, true);
