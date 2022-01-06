parts = struct();

parts.torso = {'torso_pitch', 'torso_roll', 'torso_yaw'};
parts.left_arm = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_prosup', 'l_wrist_yaw', 'l_wrist_pitch'};
parts.right_arm= {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_prosup', 'r_wrist_yaw', 'r_wrist_pitch'};
parts.left_leg = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
parts.right_leg = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};

desired = reshape(robot_logger_device.walking.joint.positions.desired.data, ...
    size(robot_logger_device.walking.joint.positions.desired.data,1), ...
    size(robot_logger_device.walking.joint.positions.desired.data,3))';

measured = reshape(robot_logger_device.walking.joint.positions.measured.data, ...
    size(robot_logger_device.walking.joint.positions.measured.data,1), ...
    size(robot_logger_device.walking.joint.positions.measured.data,3))';

retargeting = reshape(robot_logger_device.walking.joint.positions.retargeting.data, ...
    size(robot_logger_device.walking.joint.positions.retargeting.data,1), ...
    size(robot_logger_device.walking.joint.positions.retargeting.data,3))';

time = robot_logger_device.walking.joint.positions.desired.timestamps';

% close all
energy = struct();

joints_plotted = 0;

for part = fieldnames(parts)'
  figure;
  
  isNewFig = 1;
  
  hold on;
  
  l = length(parts.(part{:}));
  
  cols = 0;
  rows = 0;
  
  if(~mod(l,3))
    rows = 3;
    cols = l / 3;
  end
  
  if(~mod(l,2))
    rows = 2;
    cols = l / 2;
  end
  
  if(~mod(l,7))
    rows = 3;
    cols = 3;
  end
  
  i = 1;
  
  energy.(part{:}) = [];
  
  timestamp = time(:) - time(1);
  
  for joint = parts.(part{:})
    
    label = {};
    
    subplot(rows, cols, i)
    
    jointValue = joint{:};
    plot(timestamp, desired(:,joints_plotted+i)*180/pi, timestamp, retargeting(:,joints_plotted+i)*180/pi, timestamp, measured(:,joints_plotted+i)*180/pi)
    hold on;
    label = [label, 'desired', 'retargeting', 'measured'];

    str = regexprep(jointValue, '_', ' ');
    
    if(isNewFig)
      plot_aesthetic(str, 'time (s)' , 'Joint pos ($deg$)', '', label{:},  'Init SS Right', 'Init SS Left', 'Init DS');
%       xlim([204,210]);
      
      isNewFig = 0;
    else
      plot_aesthetic(str, 'time (s)' , 'Joint pos ($deg$)', '');
%       xlim([204,210]);
      
    end
    i = i + 1;
  end
  
  joints_plotted = l + joints_plotted;
end