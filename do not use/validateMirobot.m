%%
clear s
myObj = Mirobot_Matlab; 
s = myObj.Mirobotconnect(7); %connect to COM3 for example
disp('Finish');
pause(2);

%%
myObj.go_to_zero(s); %return back to initial position
pause(2);

%% Task 2
% % Initial position
% joint_pos_init = rad2deg([-pi/2, pi/6, -pi/6, -pi/4, 0, 0]);
% % Intermediate Pose
% joint_pos_inter = rad2deg([0, pi/3, 0, 0, 0, 0]);
% % Final Pose
% joint_pos_final = rad2deg([pi/4, pi/6, -pi/4, 0, 0, 0]);

myObj.go_to_axis(s,-90,30,-30,-45,0,0); % go to axis positoin
pause(2);

myObj.go_to_axis(s,0,60,0,0,0,0); % go to axis positoin
pause(2);

myObj.go_to_axis(s,45,30,-45,0,0,0); % go to axis positoin
pause(2);

myObj.go_to_zero(s); %return back to initial position
pause(2);

%% Task 3
myObj.go_to_cartesian_lin(s,240,00,230,0,0,0); %go to cartesian position
pause(2);

myObj.go_to_cartesian_lin(s,100,-100,265,0,0,0); %go to cartesian position
pause(2);

myObj.go_to_cartesian_lin(s,160,160,100,0,0,0); %go to cartesian position
pause(2);

myObj.go_to_zero(s); %return back to initial position
pause(2);
