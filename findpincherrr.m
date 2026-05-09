%task 5.8
arb = Arbotix('port', 'COM5', 'nservos', 5)

% arb.relax()
%arb.relax([], 'FALSE')

function [x, y, z, R] = findPincher(arb)
    servoAngles = arb.getpos(); %returns all 5 servo angles in radians
    % servoAngles = servoAngles(); %take only first four joints

    dhAngles = servo2dh(servoAngles); % convert the servo joint angles to dh joint angles using the function servo2dh
    [x,y,z,R] = pincherFK(dhAngles) % finds the current position and orientation of end-effector

end

[x, y , z, R] = findPincher(arb)