icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');
robotName='iCubGenova09';

modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
fileName='model.urdf';

consideredJoints = {'torso_pitch'; 
                    'torso_roll';
                    'torso_yaw';
                    'l_shoulder_pitch';
                    'l_shoulder_roll';
                    'l_shoulder_yaw';
                    'l_elbow';
                    'l_wrist_prosup';
                    'l_wrist_pitch';
                    'l_wrist_yaw';
                    'r_shoulder_pitch';
                    'r_shoulder_roll';
                    'r_shoulder_yaw';
                    'r_elbow';
                    'r_wrist_prosup';
                    'r_wrist_pitch';
                    'r_wrist_yaw';
                    'l_hip_pitch';
                    'l_hip_roll';
                    'l_hip_yaw';
                    'l_knee';
                    'l_ankle_pitch';
                    'l_ankle_roll';
                    'r_hip_pitch';
                    'r_hip_roll';
                    'r_hip_yaw';
                    'r_knee';
                    'r_ankle_pitch';
                    'r_ankle_roll'};

if (robot_logger_device.walking.joint.positions.desired.dimensions(1) ~= length(consideredJoints))
   error('Number of joints in consideredJoints do not match the number of joints considered by the walking.'); 
end

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(consideredJoints,'root_link',modelPath,fileName,false);

% create vector of positions
joint_pos_des = reshape(robot_logger_device.walking.joint.positions.desired.data, ...
    size(robot_logger_device.walking.joint.positions.desired.data,1), ...
    size(robot_logger_device.walking.joint.positions.desired.data,3));

% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base = [1,0,0,0;0,1,0,0;0,0,1,0.6;0,0,0,1];

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joint_pos_des(:,1),zeros(6,1),zeros(KinDynModel.NDOF,1),[0,0,-9.81]);

% Prepare figure, handles and variables required for the update, some extra
% options are commented.
% This is necessary to avoid to rely on the manual specification of meshFilePrefix used in iDynTree < 3
meshFilePrefix="";
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[1,1,1],'transparency',1);%,... % optional inputs
     %'style','wireframe','wireframe_rendering',0.1);
     
pause(0.0001);

reverseStr = '';

for i = 2 : size(joint_pos_des,2)
    % Update robot position
    % update kinematics
    iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joint_pos_des(:,i),zeros(6,1),zeros(KinDynModel.NDOF,1),[0,0,-9.81]);

    iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
    
    pause(0.0001);

    % Display the progress
    percentDone = 100 * i / size(joint_pos_des,2);
    msg = sprintf('Percentage completed: %3.1f - Sample %d/%d', percentDone, i, size(joint_pos_des,2));
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
end

fprintf('\n');
pause(1);


