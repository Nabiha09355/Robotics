%task 5.7
function dhJointAngles = servo2dh22(jointAngles)
    
    % offset: dh_angle = servo_angle + offset
    offset = [0, pi/2, 0, 0];   % in radians 
    sign  = [1, 1, 1, 1];   % depends on rotation direction
    dhJointAngles = zeros(1,4); %initializing a zero vector 

    %calculating dh joint0 angles
    dhJointAngles(1) = sign(1) .* jointAngles(1) + offset(1);
    dhJointAngles(2) = sign(2) .* jointAngles(2) + offset(2);
    dhJointAngles(3) = sign(3) .* jointAngles(3) + offset(3);
    dhJointAngles(4) = sign(4) .* jointAngles(4) + offset(4);

end

 jointangles =[-0.0051   -1.6056   -0.0205   -0.0102         0];

dhjoint = servo2dh22(jointangles)
dhjoint =[  -0.0051   -0.0348   -0.0205   -0.0102 ]
