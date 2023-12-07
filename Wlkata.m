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
pause(5);
%% Task 4
disp('Task 4');
myObj.home(s);
pause(1);
%init
myObj.go_to_axis(s,90, 53, -10, 0, -46.7, 0);
pause(1);
myObj.go_to_axis(s,90, 58, -10, 0, -46.7, 0);
pause(1);
myObj.air_pump_on(s);
pause(1);
%inter
myObj.go_to_axis(s,90, 50, -10, 0, -46.7, 0);
pause(1);
%final
myObj.go_to_axis(s,90, 70, -35, 0, -51.7, 0);
pause(1);
myObj.go_to_axis(s,90, 70, -32, 0, -58.7, 0);
pause(1);
myObj.air_pump_off(s);
pause(1);
myObj.go_to_axis(s,90, 65, -32, 0, -58.7, 0);
pause(1);
myObj.home(s);
pause(2);
%% 
load("Task4_theta_list.mat");
pump_on = 2;
pump_off = 5;

myObj.home(s);
pause(1);
%init
Task4_theta_list = Task4_theta_list*180/pi;
for i = 1:length(Task4_theta_list)
    theta = Task4_theta_list(:,i)';
    myObj.go_to_axis(s,theta(1),theta(2),theta(3),theta(4),theta(5),theta(6));
    pause(1);
    if i == pump_on
        myObj.air_pump_on(s);
        pause(1);
    end
    if i == pump_off
        myObj.air_pump_off(s);
        pause(1);
    end
end
myObj.home(s);
pause(2);

%% Task Bonus
myObj.home(s);
disp('Task bonus');
pause(10);
%% 

myObj.go_to_zero(s);
pause(2);
%% 
myObj.go_to_cartesian_lin(s,237, -101.5, 110,0,0,0);

%% 

interpolation_num = 6;
bonus_pump_on_init_1 =   [237, -101.5, 110;
                239, -70, 110;
                200, -100, 110;
                163, -71, 110;
                162, -105, 110;
                199, -67, 110];

bonus_pump_on_1 = [237, -101.5, 82;
                 238, -70, 82;
                 200, -100, 82;
                 163, -71, 82;
                 162, -105, 82;
                 199, -67, 82];

bonus_pump_on_leave_1 = [237, -101.5, 110;
                       238, -70, 110;
                       200, -100, 110;
                       163, -71, 110;
                       162, -105, 110;
                       199, -67, 110];

bonus_pump_off_init_1 = [219, 71, 110;
                       221, 103.5, 110;
                       185, 105, 110;
                       163, 90, 110;
                       184, 66, 110;
                       239, 96, 110];

bonus_pump_off_1 = [219, 71, 82;
                  221, 103.5, 82;
                  185, 105, 82;
                  163, 90, 82;
                  184, 66, 82;
                  239, 96, 82];

bonus_pump_off_leave_1 = [219, 71, 110;
                        221, 103.5, 110;
                        185, 105, 110;
                        163, 90, 110;
                        184, 68, 110;
                        239, 90, 110];

% bonus_control(s,bonus_pump_on_init_1,bonus_pump_on_1,bonus_pump_on_leave_1,bonus_pump_off_init_1,bonus_pump_off_1,bonus_pump_off_leave_1,1);


for i = 1:interpolation_num
    myObj.go_to_cartesian_lin(s,bonus_pump_on_init_1(i,1),bonus_pump_on_init_1(i,2),bonus_pump_on_init_1(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_on_1(i,1),bonus_pump_on_1(i,2),bonus_pump_on_1(i,3),0,0,0);
    pause(1);
    myObj.air_pump_on(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_on_leave_1(i,1),bonus_pump_on_leave_1(i,2),bonus_pump_on_leave_1(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_init_1(i,1),bonus_pump_off_init_1(i,2),bonus_pump_off_init_1(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_1(i,1),bonus_pump_off_1(i,2),bonus_pump_off_1(i,3),0,0,0);
    pause(1);
    if i == 5
        myObj.go_to_cartesian_lin(s,184, 68, 82,0,0,0);
        pause(1);
    end
    if  i == 6
        myObj.go_to_cartesian_lin(s,239, 90, 82,0,0,0);
        pause(1);
    end
    myObj.air_pump_off(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_leave_1(i,1),bonus_pump_off_leave_1(i,2),bonus_pump_off_leave_1(i,3),0,0,0);
    pause(1);
end

pause(5);

bonus_pump_on_init_2 = bonus_pump_off_leave_1;
bonus_pump_on_2 = bonus_pump_off_1;
bonus_pump_on_leave_2 = bonus_pump_off_init_1;
bonus_pump_off_init_2 = bonus_pump_on_leave_1;
bonus_pump_off_2 = bonus_pump_on_1;
bonus_pump_off_leave_2 = bonus_pump_on_init_1;

bonus_pump_on_init_2(5:6,:) = [184, 68, 110;239, 90, 110];
bonus_pump_on_2(5:6,:) = [186, 66, 82;239, 90, 82];
bonus_pump_on_leave_2(5:6,:) = [186, 66, 110; 239, 90, 110];
bonus_pump_off_init_2(5:6,:) = [ 162, -108, 110;  199, -61, 110];
bonus_pump_off_2(5:6,:) = [162, -108, 82; 199, -61, 82];
bonus_pump_off_leave_2(5:6,:) = [ 162, -108, 110;  199, -61, 110];


% bonus_control(s,bonus_pump_on_init_2,bonus_pump_on_2,bonus_pump_on_leave_2,bonus_pump_off_init_2,bonus_pump_off_2,bonus_pump_off_leave_2,0);
for i = 1:interpolation_num
    myObj.go_to_cartesian_lin(s,bonus_pump_on_init_2(i,1),bonus_pump_on_init_2(i,2),bonus_pump_on_init_2(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_on_2(i,1),bonus_pump_on_2(i,2),bonus_pump_on_2(i,3),0,0,0);
    pause(1);
    myObj.air_pump_on(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_on_leave_2(i,1),bonus_pump_on_leave_2(i,2),bonus_pump_on_leave_2(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_init_2(i,1),bonus_pump_off_init_2(i,2),bonus_pump_off_init_2(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_2(i,1),bonus_pump_off_2(i,2),bonus_pump_off_2(i,3),0,0,0);
    pause(1);
    myObj.air_pump_off(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,bonus_pump_off_leave_2(i,1),bonus_pump_off_leave_2(i,2),bonus_pump_off_leave_2(i,3),0,0,0);
    pause(1);
end
%% 

myObj.home(s);
pause(2);
%% 


%% end control
myObj.go_to_axis(s,0, -30, 35, 0, -90, 0);

%% 
function bonus_control(s,pump_on_init,pump_on,pump_on_leave,pump_off_init,pump_off,pump_off_leave,add)
interpolation_num = 6;
for i = 1:interpolation_num
    myObj.go_to_cartesian_lin(s,pump_on_init(i,1),pump_on_init(i,2),pump_on_init(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,pump_on(i,1),pump_on(i,2),pump_on(i,3),0,0,0);
    pause(1);
    myObj.air_pump_on(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,pump_on_leave(i,1),pump_on_leave(i,2),pump_on_leave(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,pump_off_init(i,1),pump_off_init(i,2),pump_off_init(i,3),0,0,0);
    pause(1);

    myObj.go_to_cartesian_lin(s,pump_off(i,1),pump_off(i,2),pump_off(i,3),0,0,0);
    pause(1);
    if add == 1 && i == 5
        myObj.go_to_cartesian_lin(s,184, 68, 82,0,0,0);
        pause(1);
    end
    if add == 1 && i == 6
        myObj.go_to_cartesian_lin(s,239, 90, 82,0,0,0);
        pause(1);
    end
    myObj.air_pump_off(s);
    pause(1);

    myObj.go_to_cartesian_lin(s,pump_off_leave(i,1),pump_off_leave(i,2),pump_off_leave(i,3),0,0,0);
    pause(1);
end
end
