function tip_position = cal_tip_positions(q)
d1 = 0.128;
a1 = 0.029; 
a2 = 0.108; 
a3 = 0.02; d4 = 0.168;
d6 = -0.02429;
dhparams = [0,   	   0,    	d1,      q(1);
                a1,       -pi/2,    0,        q(2);
                a2,        0,    	0,        q(3);
                a3,       -pi/2,    d4,      q(4);
                0,         pi/2,    0,        q(5);
                0,         pi/2,    d6,     q(6)];



tip_position = zeros([1,3]);
T = eye(4);
for j = 1:size(dhparams,1)
    T = T * dh_transmatrix(dhparams(j,1),dhparams(j,2),dhparams(j,3),dhparams(j,4));
end
for j = 1:3
    tip_position(1,j) = T(j,4);
end
end