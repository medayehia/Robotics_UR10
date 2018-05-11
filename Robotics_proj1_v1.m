addpath('C:\Users\M.Yehia\Desktop\V-rep with matlab');
addpath('C:\Users\M.Yehia\Desktop\V-rep with matlab\vrchk.m');

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
ClientID = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
if ClientID < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', ClientID);


RobotUR10 = struct('ClientID', ClientID);
jointNames={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4','UR10_joint5','UR10_joint6'};
ur10Joints = -ones(1,6); 
for i = 1:6
    [res, ur10Joints(i)] = vrep.simxGetObjectHandle(ClientID,jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end

RobotUR10.ur10Joints = ur10Joints;

[res, ur10Ref] = vrep.simxGetObjectHandle(ClientID, 'UR10',vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, ur10Gripper] = vrep.simxGetObjectHandle(ClientID,'UR10_connection',vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

RobotUR10.ur10Ref = ur10Ref;
RobotUR10.ur10Gripper = ur10Gripper;

[res, RobotUR10.base] = vrep.simxGetObjectHandle(ClientID,'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);


Frames = -ones(1,6); 
FramesTarget = -ones(1,6); 
Frames(1) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
Frames(2) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
Frames(3) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
Frames(4) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
Frames(5) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
Frames(6) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(1) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(2) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(3) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(4) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(5) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
FramesTarget(6) = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
RobotUR10.FrameEnd = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
RobotUR10.FrameEndTarget = copyf( ClientID, vrep, eye(4), RobotUR10.base, RobotUR10.base);
RobotUR10.FramesTarget = FramesTarget;
RobotUR10.Frames = Frames;
vrchk(vrep, res);


gRest1 = getf(ClientID, vrep, RobotUR10.ur10Joints(1), RobotUR10.base);

gRest2 = getf(ClientID, vrep, RobotUR10.ur10Joints(2), RobotUR10.base);
gRest2(1:3, 1:3) = eye(3);
gRest3 = getf(ClientID, vrep, RobotUR10.ur10Joints(3), RobotUR10.base);
gRest3(1:3, 1:3) = eye(3);
gRest4 = getf(ClientID, vrep, RobotUR10.ur10Joints(4), RobotUR10.base);
gRest4(1:3, 1:3) = eye(3);
gRest5 = getf(ClientID, vrep, RobotUR10.ur10Joints(5), RobotUR10.base);
gRest5(1:3, 1:3) = eye(3);
gRest6 = getf(ClientID, vrep, RobotUR10.ur10Joints(6), RobotUR10.base);
gRest6(1:3, 1:3) = eye(3);
gRestEnd = getf(ClientID, vrep, RobotUR10.ur10Gripper, RobotUR10.base);
gRestEnd(1:3, 1:3) = eye(3);
