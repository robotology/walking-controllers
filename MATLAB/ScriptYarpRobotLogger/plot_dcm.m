% close all;
phases;

figure

subplot(1,2,1)
plot(timestamp, dcm_pos.desired(1,:), timestamp, dcm_pos.measured(1,:))
% plotLines
plot_aesthetic('Position - X','Time (s)', 'Positon (m)', ' ', 'Desired', 'Measured', 'Init DS', 'Init SS Right', 'Init SS Left');

subplot(1,2,2)
plot(timestamp, dcm_pos.desired(2,:), timestamp, dcm_pos.measured(2,:))
% plotLines
plot_aesthetic('Position - Y','Time (s)', 'Positon (m)', ' ');

