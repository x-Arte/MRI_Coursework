classdef Mirobot_Matlab
    % Mirobot_Matlab
    % This class helps to realize basic operation and control
    % through MATLAB to MIROBOT
    %
    % It requires multifunctional extender box set to UART mode 
    methods (Static)
        function s = Mirobotconnect(COM_num)
                % Mirobotconnect Setup communication between Mirobot and
                % Matlab through serial port
                % Parameter: COM_num
                % Return: device with correct properties info
            s = serialport("COM"+COM_num,115200);
        end

        function res = readFeedback(s)
%                read feadback from serial 
               res = readline(s);
               if isstring(res) 
                   if res ~= "ok"
                       if contains(res, "Error")
                           return; 
                       end
%                        res = readline(s);
                   end
               end
        end

        function res = home(s)
                % homing Mirobot initialization command 
                % Parameter: device returned from Mirobotconnect
                % Return: return message from Mirobot
            write(s,'$h',"string");
            write(s, 'G01 F2000', "string");
            pause(5);
            res = readline(s);
            res = readline(s);
            res = readline(s);
        end

        function res = go_to_axis(s,a1,a2,a3,a4,a5,a6)
                % go_to_axis specific axis in angle mode
                % Parameter: device,a1,a2,a3,a4,a5,a6
                % Return: return message from Mirobot            
            command = "M21"+"G90 G00" + "X"+a1+"Y"+a2+"Z"+a3+"A"+a4+"B"+a5+"C"+a6+"F2000";
            write(s,command,"string");
            % readline(s); % remove one readline 
            res = Mirobot_Matlab.readFeedback(s);
        end
        
        function res = go_to_cartesian_lin(s,a1,a2,a3,a4,a5,a6)
                % go_to_cartesian_lin specific axis in coordinate mode  
                % Parameter: device,a1,a2,a3,a4,a5,a6
                % Return: return message from Mirobot                  
            command = "M20"+"G90 G00" +"X"+a1+"Y"+a2+"Z"+a3+"A"+a4+"B"+a5+"C"+a6+"F2000";
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s);
        end
        
        function res = jump_move(s,a1,a2,a3,a4,a5,a6)
                % jump_move door trajectory to specific axis in coordinate mode  
                % Parameter: device,a1,a2,a3,a4,a5,a6
                % Return: return message from Mirobot                  
            command = "M20"+"G90 G05" +"X"+a1+"Y"+a2+"Z"+a3+"A"+a4+"B"+a5+"C"+a6+"F2000";
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s);
        end
        
        function res = go_to_zero(s)
                % go_to_zero Mirobob returns to zero position 
                % Parameter: device
                % Return: return message from Mirobot             
            write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char');
            res = Mirobot_Matlab.readFeedback(s);
        end
        
        function res = air_pump_on(s)
                % air_pump_on air turn airpump on  
                % Parameter: device
                % Return: return message from Mirobot              
            write(s,'M3S1000','char')
            res = Mirobot_Matlab.readFeedback(s);
        end

        function res = air_pump_off(s)
                % air_pump_off turn airpump off   
                % Parameter: device
                % Return: return message from Mirobot               
            write(s,'M3S0','char')
            res = Mirobot_Matlab.readFeedback(s);         
        end

        function res = air_pump_blow(s)
                % air_pump_blow turn airpump blow   
                % Parameter: device
                % Return: return message from Mirobot   
            write(s,'M3S500','char')
            res = Mirobot_Matlab.readFeedback(s);       
        end

        function res = gripper_close(s)
                % gripper_close close gripper  
                % Parameter: device
                % Return: return message from Mirobot   
            write(s,'M3S60','char')
            res = Mirobot_Matlab.readFeedback(s); 
        end

        function res = gripper_open(s)
                % gripper_open open gripper  
                % Parameter: device
                % Return: return message from Mirobot   
            write(s,'M3S40','char')
            res = Mirobot_Matlab.readFeedback(s); 
        end
        
        function res = increment_axis(s,a1,a2,a3,a4,a5,a6)
                % increment_axis relative movement under angle mode  
                % Parameter: device,a1,a2,a3,a4,a5,a6
                % Return: return message from Mirobot   
            command = "M21 G91 G01"+"X"+a1+"Y"+a2+"Z"+a3+"A"+a4+"B"+a5+"C"+a6;
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s); 
        end
        
        function res = move_to_axis(s,joint_num,revolve,d)
                % move_to_axis rotate joint cw or ccw with specific degree
                % Parameter: device,joint_num,revolve,d
                % Return: return message from Mirobot   
                % api.move_to_axis(MirobotJoint.Joint1, RevolveDirection.cw, 0)
            if joint_num == 1
                joint = "X";
            elseif joint_num == 2
                joint = "Y";
            elseif joint_num == 3
                joint = "Z";
            elseif joint_num == 4
                joint = "A";
            elseif joint_num == 5
                joint = "B";
            elseif joint_num == 6
                joint = "C";
            else
                error("Invalid Joint Number");
            end
            if revolve == "cw"
                num = d;
            elseif revolve == "ccw"
                num = -d;
            else
                error("Invalid Revolultion Rirection");
            end
            % command = "M21 G91 G01" + joint + num;
            command = "M21 G91 G01" + joint + num + "F2000";
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s); 
        end
        
        function res = increment_cartesian_lin(s,a1,a2,a3,a4,a5,a6)
                % increment_cartesian_lin relative movement in coordinate mode  
                % Parameter: device,a1,a2,a3,a4,a5,a6
                % Return: return message from Mirobot   
            command = "M20 G91 G00" + "X"+a1+"Y"+a2+"Z"+a3+"A"+a4+"B"+a5+"C"+a6;
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s); 
        end
        
        function res = direction_mobility(s,direction,distance)
                % direction_mobility move in one direction with specific distance  
                % Parameter: device,direction,distance
                % Return: return message from Mirobot   
            %api.direction_mobility(MoveDirection.forward, 0)
            if direction == "forward"
                direction = "Y";
            elseif direction == "backward"
                direction = "Y";
                distance = -distance;
            elseif direction == "up"
                direction = "Z";
            elseif direction == "down"
                direction = "Z";
                distance = -distance;                
            elseif direction == "right"
                direction = "X";              
            elseif direction == "left"
                direction = "X";
                distance = -distance;                  
            end
            command = "M20 G91 G01"+direction+distance;
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s);              
        end
        
        function res = slider_move_to(s, distance, speed)
                % slider_move_to move slider with specific distance and speed  
                % Parameter: device,distance,speed
                % Return: return message from Mirobot   
            command = "G90 G01 "+ "D" + distance + "F" + speed;            
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s);             
        end
        
        function res = conveyor_move_to(s, distance, speed)
                % conveyor_move_to move conveyor belt with specific distance and speed   
                % Parameter: device,distance,speed
                % Return: return message from Mirobot   
            command = "G90 G01 "+ "D" + distance + "F" + speed;     
            write(s,command,"string");
            res = Mirobot_Matlab.readFeedback(s);             
        end
        
        function res = set_7th_axis(s, mode)
            % set_7th_axis set 7th axis mode to conveyor or rail
            % Parameter: device, mode
            % Return: return message from Mirobot   
            % $45=1 (Conveyor mode 1:Conveyor mode. 0:Rail mode)
            % $50=0 (0: no tool, 1: suction cup, 2: grip, 3: soft claw (finger),
            % 4: soft claw (ball), 5: Custom)
            if lower(mode) == "conveyor"
                write("$45 = 1" + newline + "$50 = 0");
            elseif lower(mode) == "rail"
                write("$45 = 0" + newline + "$50 = 0");
            end 
            res = Mirobot_Matlab.readFeedback(s); 
        end
    end
end