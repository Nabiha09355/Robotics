function [x, y, z, R] = pincherFK_proper(jointAngles)
    % pincherFK: Calculates Forward Kinematics for Phantom X Pincher 
    %
    % Inputs:
    %   jointAngles - 1x4 vector of DH joint angles [th1, th2, th3, th4] in radians 
    %
    % Outputs:
    %   x, y, z - Cartesian position of the gripper in mm 
    %   R       - 3x3 Rotation matrix representing end-effector orientation 

    % --- 1. Physical Parameters (from Task 5.5) ---
    d1 = 145; 
    a2 = 105;
    a3 = 106;
    a4 = 75;

    % --- 2. Extract Angles ---
    th1 = jointAngles(1);
    th2 = jointAngles(2);
    th3 = jointAngles(3);
    th4 = jointAngles(4);

    % --- 3. DH Transformation Helper ---
    % Standard DH formula 
    dh = @(theta, d, a, alpha) [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),            d;
        0,           0,                      0,                     1
    ];

    % --- 4. Link Transformations ---
    T01 = dh(th1, d1, 0, pi/2);
    T12 = dh(th2, 0, a2, 0);
    T23 = dh(th3, 0, a3, 0);
    T34 = dh(th4, 0, a4, 0);

    % --- 5. Full Transformation Matrix ---
    % Multiplied in order from base to end-effector 
    T04 = T01 * T12 * T23 * T34;

    % --- 6. Extract Outputs ---
    x = T04(1,4);
    y = T04(2,4);
    z = T04(3,4);
    R = T04(1:3, 1:3); % Rotation matrix is the top-left 3x3 sub-matrix 
end
