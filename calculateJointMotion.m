function [positions, velocities] = calculateJointMotion(th0, thf, Step)
%function to cal positions and velocities
    a0 = th0;
    a1 = 0;
    a2 = 3*(thf-th0)/Step^2;
    a3 = -2*(thf-th0)/Step^3;
    t = 0:1:Step;
    positions = a0 + a1*t + a2*t.^2 + a3*t.^3;
    velocities = a1 + 2*a2*t + 3*a3*t.^2;
end