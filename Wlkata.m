%% 

clear s
myObj = Mirobot_Matlab; 
s = myObj.Mirobotconnect(5); %connect to COM3 for example
disp('Finish');
pause(2);

%%
myObj.go_to_zero(s); %return back to initial position
pause(2);

% %% Task 2
% % % Initial position
% % joint_pos_init = rad2deg([-pi/2, pi/6, -pi/6, -pi/4, 0, 0]);
% % % Intermediate Pose
% % joint_pos_inter = rad2deg([0, pi/3, 0, 0, 0, 0]);
% % % Final Pose
% % joint_pos_final = rad2deg([pi/4, pi/6, -pi/4, 0, 0, 0]);
% 
% myObj.go_to_axis(s,-90,30,-30,-45,0,0); % go to axis positoin
% pause(2);
% 
% myObj.go_to_axis(s,0,60,0,0,0,0); % go to axis positoin
% pause(2);
% 
% myObj.go_to_axis(s,45,30,-45,0,0,0); % go to axis positoin
% pause(2);
% 
% myObj.go_to_zero(s); %return back to initial position
% pause(2);

%% Task 3
myObj.go_to_zero(s); %return back to initial position
pause(2);
% %test positon
% myObj.go_to_axis(s,0,23.1,-25.7,0,2.6,0); %init
% pause(1);
% 
% myObj.go_to_axis(s,-44.9,-25.1,9.7,0,15.4,44.9); %inter
% pause(1);
% 
% myObj.go_to_axis(s,44.9,31.0,9.8,0,-40.8,-44.9); %final 
% pause(3);
% 
% myObj.go_to_zero(s); %return back to initial position
% pause(1);

disp('Task 3');
load("Task3_theta_list.mat");
Task3_theta_list = Task3_theta_list*180/pi;
for i = 1:length(Task3_theta_list)
    theta = Task3_theta_list(:,i)';
    myObj.go_to_axis(s,theta(1),theta(2),theta(3),theta(4),theta(5),theta(6));
    if i == 1
        pause(1)
    end
    pause(0.2);
end
%% Task 4
disp('Task 4');
myObj.go_to_zero(s); %return back to initial position
pause(2);
%init
%
myObj.go_to_axis(s,90, 20, -3.7, 0,  -43.7, 0);
pause(2);
% 0, 250, 55
myObj.go_to_axis(s,90, 56.9, -13.2, 0, -43.7, 0);
pause(2);
myObj.air_pump_on(s);
pause(2);
%inter
% 
myObj.go_to_axis(s,90, 29.7, -34.4, 0, 0, 0);
pause(2);

%final
%0, 300, 55
myObj.go_to_axis(s,90, 69.0, -35.5, 0, -48, 0);
pause(2);
%
myObj.air_pump_off(s);

%% 
myObj.go_to_zero(s);
pause(5);


%% end control
myObj.go_to_axis(s,0, -30, 35, 0, -90, 0);


