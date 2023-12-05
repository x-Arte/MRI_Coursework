function  [theta_result,Q_k_1] = interative_ik(init_joint,target_position,transformation_matrix)
    syms th1 th2 th3 th4 th5 th6 

    T = transformation_matrix;    
    Q_t = T(1:3,4);                % Translation Vector
    JQ = jacobian(Q_t,[th1,th2,th3,th4,th5,th6]);   % Jacobian Matrix  3 by 3
    
    % Initial interative
    theta_k_1 = init_joint;                   

    %% Newton Raphson's loop
%     theta_limitation = [-pi, pi;      %Angle constraints
%          -pi/2, pi/2;
%          -pi/2, pi/2;
%          -pi, pi;
%          0, 0;
%          0, 0];
%     theta_limitation = [-pi*110/180, pi;      %Angle constraints
%          -pi/2, pi/2;
%          -pi/2, pi/2;
%          -pi, pi;
%          0, 0;
%          0, 0];
    theta_limitation = [-100, 160; %Angle constraints
                    -40-90, 70-90;
                    -170, 60;
                    -350, 350; 
                    -205+90, 36+90;
                    -360, 360];
    theta_limitation = theta_limitation*pi/180;

    Q1 = subs(Q_t,th1,theta_k_1(1));
    Q2 = subs(Q1,th2,theta_k_1(2));
    Q3 = subs(Q2,th3,theta_k_1(3));
    Q4 = subs(Q3,th4,theta_k_1(4));
    Q5 = subs(Q4,th5,theta_k_1(5));
    Q6 = subs(Q5,th6,theta_k_1(6));
    Q_k_1 = double(vpa(Q6,12)); % Translation vector for theta_k_1, Convert from symbolic to numeric 
    
    Vp_to_target = target_position-Q_k_1; % vector from guessed position to desired target position 
    p_to_target = norm(Vp_to_target); % Distance between target position and guess 
    
    % interative begin
    while p_to_target > 0.0001
        
        J0 = subs(JQ,th1,theta_k_1(1));
        J1 = subs(J0,th2,theta_k_1(2));
        J2 = subs(J1,th3,theta_k_1(3));
        J3 = subs(J2,th4,theta_k_1(4));
        J4 = subs(J3,th5,theta_k_1(5));
        J5 = subs(J4,th6,theta_k_1(6));
        Jacob = double(vpa(J5,12));       % Jacobian for theta_k_1, Convert from symbolic to numeric       
        Jinv = pinv(Jacob);
    
        theta_k = theta_k_1 + Jinv * Vp_to_target;     % Using the iterative method
        
        for i = 1:size(theta_k,1)                                        % Angle Constrains 
            theta_k(i) = max(min(theta_k(i), theta_limitation(i,2)),theta_limitation(i,1));    
        end
%         theta_list
%         theta_k
        theta_list = [theta_list, theta_k];     
    
        Q1 = subs(Q_t,th1,theta_k(1));
        Q2 = subs(Q1,th2,theta_k(2));
        Q3 = subs(Q2,th3,theta_k(3));
        Q4 = subs(Q3,th4,theta_k(4));
        Q5 = subs(Q4,th5,theta_k(5));
        Q6 = subs(Q5,th6,theta_k(6));
        Q_k = double(vpa(Q6,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
           
        Vp_to_target = target_position-Q_k;         % vector from estimated position to desired target position
        p_to_target = norm(Vp_to_target);     
        
        theta_k_1 = theta_k; 
        Q_k_1 = Q_k;   
    end
theta_result = theta_k_1;
end
