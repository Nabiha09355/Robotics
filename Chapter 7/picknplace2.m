global arb;

% 1. DEFINE DESTINATION TARGETS
% Coordinates for where the object SHOULD GO (Place)
x_place = 0; 
y_place = 150; 
z_hover = 140; % Transit height
z_grasp = 50;  % Placement height
phi = -pi/2;

% 2. STARTUP: OPEN JAW AND RELAX
disp('Opening jaws and relaxing arm...');
positionJaw(33); 
pause(1);

% Turn off torque so you can move the arm
arb.relax();
input('Manually move the arm to the object and hold it. Press Enter to grab...', 's');

% 3. CAPTURE AND RE-ENGAGE
% Read the manual position (in ticks)
q_pick_ticks = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];

% Re-engage torque to lock the arm in its current position
arb.setpos([q_pick_ticks, arb.getpos(5)], [20,20,20,20,20]);
pause(0.5);

% Convert ticks to radians for the math functions
q_pick_rads = servo2dh(q_pick_ticks);

disp('Action: Grabbing object...');
positionJaw(24); % Grab the object
pause(1);

% 4. CALCULATE PATH FROM THE MANUAL POSITION
disp('Calculating safe lift and transit path...');

% Find the Cartesian X and Y of where you manually placed the arm
[x_now, y_now, ~, ~] = pincherFK(q_pick_rads);

% Plan the trajectory using the manual spot as the starting point
q_lift        = findSolution(x_now, y_now, z_hover, phi, q_pick_rads);
q_place_hover = findSolution(x_place, y_place, z_hover, phi, q_lift);
q_place_drop  = findSolution(x_place, y_place, z_grasp, phi, q_place_hover);

% 5. EXECUTE MOVE TO PLACE
disp('Lifting object...');
setPosition(q_lift);
pause(2);

disp('Moving to destination...');
setPosition(q_place_hover);
pause(2);

disp('Lowering to place location...');
setPosition(q_place_drop);
pause(1);

disp('Action: Releasing object...');
positionJaw(33); 
pause(1);

% Clear the object before finishing
setPosition(q_place_hover);
disp('Task Completed.');
