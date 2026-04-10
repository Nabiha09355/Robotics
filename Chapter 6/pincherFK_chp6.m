function [x, y, z, R] = pincherFK_chp6(jointAngles)
    % Physical Link Lengths (mm)
    d1 = 145; 
    a2 = 105; 
    a3 = 106; 
    a4 = 75;

    th1 = jointAngles(1);
    th2 = jointAngles(2);
    th3 = jointAngles(3);
    th4 = jointAngles(4);

    % Algebraic Equations from Lab Handbook 6.4.2
    x = cos(th1) * (a2*cos(th2) + a3*cos(th2+th3) + a4*cos(th2+th3+th4));
    y = sin(th1) * (a2*cos(th2) + a3*cos(th2+th3) + a4*cos(th2+th3+th4));
    z = d1 + a2*sin(th2) + a3*sin(th2+th3) + a4*sin(th2+th3+th4);

    % Dummy rotation matrix to satisfy function outputs for now
    R = eye(3); 
end