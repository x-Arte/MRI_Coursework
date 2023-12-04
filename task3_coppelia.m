function task3_coppelia()
% This function can build the connection to CoppeliaSim and control the
% robot on its joint space
% This script should only be used for tutorial sessions of Medical Robotics and Instrumentation
% It is based on open source code repo and modified by Kaizhong
% Modified by Mengyi Zhou 2023/12

disp('Program started');
% Import library functions
vrep=remApi('remoteApi');
% End the communication thread
vrep.simxFinish(-1);
% Start communication connection
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
% Determine whether the communication is successful
if id < 0
disp('Failed connecting to remote API server. Exiting.');
vrep.delete();
return;
end
fprintf('Connection %d to remote API server open.\n', id);
% When the program is interrupted, clear all between connections MATLAB and V-rep 
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
% This will only work in "continuous remote API server service"
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
% Start simulation
res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
% Initialize all objects that need to be manipulated
disp('Robot initialising');
%% 
h = tutorial_mirobot_control_init(vrep, id);

%% 
pause(.2);

%%% Calculating the joint motion path in joint space
load("Task3_theta_list.mat");
pos = [];

for i = 1:length(Task3_theta_list)
    this_pos = Task3_theta_list(:,i)';
    this_pos = transform_angle(this_pos);
    pos = [pos;this_pos];    
end
for j = 2:length(Task3_theta_list)
    dt = 0.05; % Time of each simulation step
    executing_time = 0.09;
    t_step  = 0:dt:executing_time;
    step_pos = interp1([0, dt],[pos(j-1,:);pos(j,:)],t_step); % Linear interpolation
    disp('Jogging the robot by directly setting joint position');
    for steps = 1:length(t_step)
    % The communication pause allows all commands to be sent simultaneously
        vrep.simxPauseCommunication(id, 1);  % Pause the signal sending
        for i = 1:1:6
            res = vrep.simxSetJointPosition(id, h.Joints(i), step_pos(steps,i), vrep.simx_opmode_oneshot); % Set joint position
            vrchk(vrep, res, true);
        end
        vrep.simxPauseCommunication(id, 0);  % Continue signal sending
        pause(dt);  % Allow time for the simulator to run
    
    [res, position] = vrep.simxGetObjectPosition(id, h.Tip, h.Base,vrep.simx_opmode_oneshot); vrchk(vrep, res, true);
    end
end
%%% Set robot joint position in simulation
disp("End jogging");

% Pause for 5s
pause(2);   
end
%% 
function handles = tutorial_mirobot_control_init(vrep, id)
% This function can initiate the robot object in the CoppeliaSim
% This script should only be used for tutorial sessions of Medical Robotics and Instrumentation
% It is based on open source code repo and modified by Kaizhong
handles = struct('id', id);

Joints = [-1,-1,-1,-1,-1,-1]; 

% Get the handle of Joints
[res Joints(1)] = vrep.simxGetObjectHandle(id, 'joint1', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Joints(2)] = vrep.simxGetObjectHandle(id, 'joint2', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Joints(3)] = vrep.simxGetObjectHandle(id, 'joint3', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Joints(4)] = vrep.simxGetObjectHandle(id, 'joint4', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Joints(5)] = vrep.simxGetObjectHandle(id, 'joint5', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Joints(6)] = vrep.simxGetObjectHandle(id, 'joint6', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.Joints = Joints;

% Get the handle of robot_tip
[res Tip] = vrep.simxGetObjectHandle(id, 'Tip', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
handles.Tip = Tip;

% Get the handle of the robot base
[res Base] = vrep.simxGetObjectHandle(id, 'Mirobot', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
handles.Base = Base;


% Get feedback value for the first time
for i = 1:6
  res = vrep.simxGetJointPosition(id, Joints(i), vrep.simx_opmode_streaming); vrchk(vrep, res, true);
end

res = vrep.simxGetObjectPosition(id, Tip, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, Tip, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

end
