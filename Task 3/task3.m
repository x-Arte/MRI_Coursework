clc;
close all;
clear;

%% Setup Robot
robot = rigidBodyTree('DataFormat', 'column');
base = robot.Base;

arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
arm4 = rigidBody("arm4");
arm5 = rigidBody("arm5");
endeffector = rigidBody("endeffector");

coll1 = collisionCylinder(0.03,0.128);    % cylinder: radius,length
coll1.Pose = trvec2tform([0 0 -0.127/2]); % centre
coll2 = collisionBox(0.128,0.03,0.03);  % box:length,width,height (x,y,z)
coll2.Pose = trvec2tform([0.108/2 0 0]);  % centre
coll3 = collisionBox(0.02,0.168, 0.03); % box:length,width,height (x,y,z)
coll3.Pose = trvec2tform([0.02 -0.16898/2 0]); % centre
coll4 = collisionCylinder(0.015,0.03); % cylinder: radius,length
coll4.Pose = trvec2tform([0 0 0]); % centre
coll5 = collisionBox(0.02,0.02429,0.02); % box:length,width,height (x,y,z)
coll5.Pose = trvec2tform([0 -0.02429/2 0]); % centre

numSides = 6;
radius = 0.015;
height = 0.05;
baseX = radius * cos(2 * pi * (1:numSides) / numSides);
baseY = radius * sin(2 * pi * (1:numSides) / numSides);
baseZ = zeros(1, numSides);
tip = [0, 0, height]; 
tribox = [baseX; baseY; baseZ]';
tribox = [tribox; tip];
collEndeffector = collisionMesh(tribox);
collEndeffector.Pose = trvec2tform([0 0 0]);


addCollision(arm1,coll1)           % add Collision to the links
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(arm4,coll4)
addCollision(arm5,coll5)
addCollision(endeffector,collEndeffector)

% set joints as revolute joints  
jnt1 = rigidBodyJoint("jnt0","revolute");
jnt2 = rigidBodyJoint("jnt1","revolute");
jnt3 = rigidBodyJoint("jnt2","revolute");
jnt4 = rigidBodyJoint("jnt3","revolute");
jnt5 = rigidBodyJoint("jnt4","revolute");
jntend = rigidBodyJoint("endeffector_joint","revolute");

dhparams = [0   	  0    	  0.127  	    0;
            0.02969   pi/2    0             pi/2;
            0.108     0       0             0;
            0.02      pi/2    0.16898       0;
            0        -pi/2    0            -pi/2;
            0         pi/2    0.02429       0];

bodies = {base,arm1,arm2,arm3,arm4,arm5,endeffector};   
joints = {[],jnt1,jnt2,jnt3,jnt4,jnt5,jntend};

% figure("Name","Robot","Visible","on")               % Visualise the robot
for i = 2:length(bodies)
    setFixedTransform(joints{i},dhparams(i-1,:),"mdh");
    bodies{i}.Joint = joints{i};
    addBody(robot,bodies{i},bodies{i-1}.Name)
%     show(robot,"Collisions","on","Frames","off");
%     drawnow;     
end

config = homeConfiguration(robot);
inputs = [0, 0, 0, 0, 0, 0];
for i = 1:length(joints)-1
    config(i) = dhparams(i,4) + inputs(i);
end
% figure;
% show(robot,config);
% figure("Name","Robot","Visible","on")
% show(robot,config,"Collisions","on","Frames","off");

%% Forward Kinematics
syms th1 th2 th3 th4 th5 th6  % Do not forget to install "Symbolic Math Toolbox"

a = dhparams(:,1);
alpha = dhparams(:,2);
d = dhparams(:,3);
theta = dhparams(:,4) + [th1 th2 th3 th4 th5 th6]';

%Transformation matrix
T = dh_to_transform_matrix(a, alpha, d, theta);
T = simplify(T);

%%  Desired end-effector position
p_initial=[0.24,0,0.23]';
p_Intermediate=[0.1,-0.1,0.265]';
p_final=[0.16,0.16,0.1]';

%% Interative IK  (Newton Raphson's loop)
theta_initial = interative_ik(p_initial,T);
theta_inter = interative_ik(p_Intermediate,T);
theta_final = interative_ik(p_final,T);

%% Calculate Linear Planning Paths
t = 0.01:0.01:0.5;
Qd1 = p_Intermediate - p_initial;
Qd2 = p_final - p_Intermediate;
theta_initial2inter = [];
theta_inter2final = [];
p_interpolation=[];
p_interpolation=[p_interpolation,p_initial];
theta_initial2inter = [theta_initial2inter,theta_initial];
for i = 1:length(t)
    p_interpolation1 = p_initial + Qd1 / length(t) * i;
    p_interpolation = [p_interpolation,p_interpolation1];
    theta_interpolation1 = interative_ik(p_interpolation1,T);
    theta_initial2inter = [theta_initial2inter,theta_interpolation1];
    fprintf("Iteration: %d\n", i);
end
for i = 1:length(t)
    p_interpolation2 = p_Intermediate + Qd2 / length(t) * i;
    p_interpolation = [p_interpolation,p_interpolation2];
    theta_interpolation2 = interative_ik(p_interpolation2,T);
    theta_inter2final = [theta_inter2final,theta_interpolation2];
    fprintf("Iteration: %d\n", i);
end
theta_initial2inter = theta_initial2inter + dhparams(:,4);
theta_inter2final = theta_inter2final + dhparams(:,4);
theta_initial2final = [theta_initial2inter,theta_inter2final];

figure;
plot(0:0.01:1.0, theta_initial2final);
title("Position of Each Joint");
xlabel("Time (s)");
ylabel("Position (rad)");
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'End Effector');


% Calculate speed
speedMatrix = diff(theta_initial2final) ./ diff((1:size(theta_initial2final, 1))');
speedMatrix = [NaN(1, size(theta_initial2final, 2)); speedMatrix];

figure
plot(0:0.01:1.0,speedMatrix);
title("Speed of Each Joint");
xlabel("Time (s)");
ylabel("Speed (rad/s)");
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'End Effector');


% Plot the trajectory of the robot tip
p_x = p_interpolation(1,:);
p_y = p_interpolation(2,:);
p_z = p_interpolation(3,:);

% plot trajectory
figure;
% plot3(p_x, p_y, p_z, '-');
plot3(p_x(1), p_y(1), p_z(1), 'o');
title('Trajectory of the Robot Tip');
xlabel('X Axes');
ylabel('Y Axes');
zlabel('Z Axes');
grid on;

 %% Robot Simulation & Creating Video
Initial = theta_initial + dhparams(:,4);
Inter = theta_inter + dhparams(:,4);
Final = theta_final + dhparams(:,4);

Step = 50;
framesPerSecond = 5;
Qd = Inter - Initial;
for i = 1:Step
    q = Initial + Qd / Step * i;
    show(robot,q,"Collisions","on","Frames","off");
    F(i) = getframe(gcf) ;
    drawnow
end

Qd = Final - Inter;
for i = 1:Step
    q = Inter + Qd / Step * i;
    show(robot,q,"Collisions","on","Frames","off");
    G(i) = getframe(gcf);
    drawnow
end

F = [F, G];
% create the video writer with 1 fps
writerObj = VideoWriter('task3.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

%% 
save("theta_initial2final.mat","theta_initial2final");

