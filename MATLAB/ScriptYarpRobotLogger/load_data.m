%% Setup the Import Options and import the data
time = robot_logger_device.walking.joint.positions.desired.timestamps';

timestamp = time - time(1);

% Load joint positions
parts = struct();

parts.torso = {'torso_pitch', 'torso_roll', 'torso_yaw'};
parts.left_arm = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_prosup', 'l_wrist_yaw', 'l_wrist_pitch'};
parts.right_arm= {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_prosup', 'r_wrist_yaw', 'r_wrist_pitch'};
parts.left_leg = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
parts.right_leg = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};

joint_pos.desired = robot_logger_device.walking.joint.positions.desired.data;

joint_pos.measured = robot_logger_device.walking.joint.positions.measured.data;

joint_pos.retargeting = robot_logger_device.walking.joint.positions.retargeting.data;


% Load joint velocities
joint_vel.measured = robot_logger_device.walking.joint.velocities.measured.data;


% Load zmp
zmp.desired_planner = robot_logger_device.walking.zmp.desired_planner.data;

zmp.desired = robot_logger_device.walking.zmp.measured.data;

zmp.measured = robot_logger_device.walking.zmp.desired.data;


% Load CoM position
com_pos.desired = robot_logger_device.walking.com.position.desired.data;

com_pos.desired_macumba = robot_logger_device.walking.com.position.desired_macumba.data;

com_pos.measured = robot_logger_device.walking.com.position.measured.data;


% Load CoM position
com_vel.desired = robot_logger_device.walking.com.velocity.desired.data;


% Load DCM pos
dcm_pos.desired = robot_logger_device.walking.dcm.position.desired.data;

dcm_pos.measured = robot_logger_device.walking.dcm.position.measured.data;


% Load DCM vel
dcm_vel.desired = robot_logger_device.walking.dcm.velocity.desired.data;


% Load right foot
right_foot_pos.desired = robot_logger_device.walking.right_foot.position.desired.data;
right_foot_pos.measured = robot_logger_device.walking.right_foot.position.measured.data;

right_foot_orientation.desired = robot_logger_device.walking.right_foot.orientation.desired.data;
right_foot_orientation.measured = robot_logger_device.walking.right_foot.orientation.measured.data;

right_foot_lin_vel.measured = robot_logger_device.walking.right_foot.linear_velocity.measured.data;
right_foot_ang_vel.measured = robot_logger_device.walking.right_foot.angular_velocity.measured.data;

right_foot_lin_force.measured = robot_logger_device.walking.right_foot.linear_force.measured.data;
right_foot_ang_torque.measured = robot_logger_device.walking.right_foot.angular_torque.measured.data;


% Load left foot
left_foot_pos.desired = robot_logger_device.walking.left_foot.position.desired.data;
left_foot_pos.measured = robot_logger_device.walking.left_foot.position.measured.data;

left_foot_orientation.desired = robot_logger_device.walking.left_foot.orientation.desired.data;
left_foot_orientation.measured = robot_logger_device.walking.left_foot.orientation.measured.data;

left_foot_lin_vel.measured = robot_logger_device.walking.left_foot.linear_velocity.measured.data;
left_foot_ang_vel.measured = robot_logger_device.walking.left_foot.angular_velocity.measured.data;

left_foot_lin_force.measured = robot_logger_device.walking.left_foot.linear_force.measured.data;
left_foot_ang_torque.measured = robot_logger_device.walking.left_foot.angular_torque.measured.data;
