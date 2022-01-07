phases;

%% Right foot

figure

subplot(2,3,1)
plot(timestamp, right_foot_pos.desired(1,:), timestamp, right_foot_pos.measured(1,:))
%plotLines;
plot_aesthetic('Right Foot - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init DS', 'Init SS Right', 'Init SS Left');

subplot(2,3,2)
plot(timestamp, right_foot_pos.desired(2,:), timestamp, right_foot_pos.measured(2,:))
%plotLines;
plot_aesthetic('Right Foot - Y','Time (s)', 'Positon (m)', ' ')


subplot(2,3,3)
plot(timestamp, right_foot_pos.desired(3,:), timestamp, right_foot_pos.measured(3,:))
%plotLines;
plot_aesthetic('Right Foot - Z','Time (s)', 'Positon (m)', ' ')


subplot(2,3,4)
plot(timestamp, 180/pi * right_foot_orientation.desired(1,:), timestamp, 180/pi * right_foot_orientation.measured(1,:))
%plotLines;
plot_aesthetic('Right Foot - Roll','Time (s)', 'Angle (deg)', ' ')


subplot(2,3,5)
plot(timestamp, 180/pi * right_foot_orientation.desired(2,:), timestamp, 180/pi * right_foot_orientation.measured(2,:))
%plotLines;
plot_aesthetic('Right Foot - Pitch','Time (s)', 'Angle (deg)', ' ')


subplot(2,3,6)
plot(timestamp, 180/pi * right_foot_orientation.desired(3,:), timestamp, 180/pi * right_foot_orientation.measured(3,:))
%plotLines;
plot_aesthetic('Right Foot - Yaw','Time (s)', 'Angle (deg)', ' ');


%% Left foot

figure

subplot(2,3,1)
plot(timestamp, left_foot_pos.desired(1,:), timestamp, left_foot_pos.measured(1,:))
%plotLines;
plot_aesthetic('Left Foot - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init DS', 'Init SS right', 'Init SS Left');

subplot(2,3,2)
plot(timestamp, left_foot_pos.desired(2,:), timestamp, left_foot_pos.measured(2,:))
%plotLines;
plot_aesthetic('Left Foot - Y','Time (s)', 'Positon (m)', ' ')


subplot(2,3,3)
plot(timestamp, left_foot_pos.desired(3,:), timestamp, left_foot_pos.measured(3,:))
%plotLines;
plot_aesthetic('Left Foot - Z','Time (s)', 'Positon (m)', ' ')


subplot(2,3,4)
plot(timestamp, 180/pi * left_foot_orientation.desired(1,:), timestamp, 180/pi * left_foot_orientation.measured(1,:))
%plotLines;
plot_aesthetic('Left Foot - Roll','Time (s)', 'Angle (deg)', ' ')


subplot(2,3,5)
plot(timestamp, 180/pi * left_foot_orientation.desired(2,:), timestamp, 180/pi * left_foot_orientation.measured(2,:))
%plotLines;
plot_aesthetic('Left Foot - Pitch','Time (s)', 'Angle (deg)', ' ')


subplot(2,3,6)
plot(timestamp, 180/pi * left_foot_orientation.desired(3,:), timestamp, 180/pi * left_foot_orientation.measured(3,:))
%plotLines;
plot_aesthetic('Left Foot - Yaw','Time (s)', 'Angle (deg)', ' ');