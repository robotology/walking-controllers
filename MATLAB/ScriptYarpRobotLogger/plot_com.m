figure

phases;

% com_ik_dy = -1.0 * (zmp_des_y - zmp_y) + 5 * (com_des_y - com_y) + com_des_dy;
% com_ik_dx = -1.0 * (zmp_des_x - zmp_x) + 5 * (com_des_x - com_x) + com_des_dx;
% 
% com_ik_x = cumtrapz(com_ik_dx) * 0.01 + com_x(1);
% com_ik_y = cumtrapz(com_ik_dy) * 0.01 + com_y(1);

subplot(1,3,1)
plot(timestamp, com_pos.desired(1,:), timestamp, com_pos.desired_macumba(1,:), timestamp, com_pos.measured(1,:))
% plotLines;
plot_aesthetic('CoM - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Desired Macumba', 'Measured');

subplot(1,3,2)
plot(timestamp, com_pos.desired(2,:), timestamp, com_pos.desired_macumba(2,:), timestamp, com_pos.measured(2,:))
% plotLines

plot_aesthetic('CoM - Y','Time (s)', 'Positon (m)', '');


subplot(1,3,3)
plot(timestamp, com_pos.desired(3,:), timestamp, com_pos.desired_macumba(3,:), timestamp, com_pos.measured(3,:))

plot_aesthetic('CoM - Z','Time (s)', 'Positon (m)', '');

