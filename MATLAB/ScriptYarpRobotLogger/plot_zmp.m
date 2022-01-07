% close all;

figure

phases;

subplot(1,2,1)
plot(timestamp, zmp.desired_planner(1,:), timestamp, zmp.desired(1,:), timestamp, zmp.measured(1,:))
plotLines;
plot_aesthetic('ZMP - X','Time (s)', 'Positon (m)', ' ', 'Desired Planner', 'Desired', 'Measured', 'Init DS', 'Init SS Right', 'Init SS Left');

subplot(1,2,2)
plot(timestamp, zmp.desired_planner(2,:), timestamp, zmp.desired(2,:), timestamp, zmp.measured(2,:))
plotLines;
plot_aesthetic('ZMP - Y','Time (s)', 'Positon (m)', ' ');

foot_offset = [0.063 0.041];
footSize = [0.19, 0.09];
