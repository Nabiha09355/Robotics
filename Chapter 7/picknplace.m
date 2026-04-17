global arb;

% ==========================================
% 1. DEFINE TARGETS (Update these values)
% ==========================================
% Coordinates for where the object IS (Pick)
x_pick = 150; y_pick = 50; 

% Coordinates for where the object SHOULD GO (Place)
x_place = 0; y_place = 150; 

% Safe Heights (Using your calibrated values)
z_hover = 140; 
z_grasp = 50; 
phi = -pi/2;

% ==========================================
% 2. STARTUP: FULLY OPEN JAW
% ==========================================
disp('Initializing: Opening jaws...');
positionJaw(33); % Setting jaw to fully open position 
pause(1);

% ==========================================
% 3. CALCULATE INVERSE KINEMATICS
% ==========================================
disp('Calculating safe path...');
current_q = [arb.getpos(1), arb.getpos(2), arb.getpos(3), arb.getpos(4)];

% Find solutions for the Pick location [cite: 1233]
q_pick_hover = findSolution(x_pick, y_pick, z_hover, phi, current_q);
q_pick_grasp = findSolution(x_pick, y_pick, z_grasp, phi, q_pick_hover);

% Find solutions for the Place location
q_place_hover = findSolution(x_place, y_place, z_hover, phi, q_pick_hover);
q_place_grasp = findSolution(x_place, y_place, z_grasp, phi, q_place_hover);

% ==========================================
% 4. EXECUTE THE PICK
% ==========================================
disp('Moving to object...');
setPosition(q_pick_hover); % Approach from above [cite: 1329, 1365]
pause(2);
setPosition(q_pick_grasp); % Lower to the object
pause(1);

disp('Action: Grabbing object...');
positionJaw(24); % Contract jaws to grab (Adjust 24 to your cube width)
pause(1);

setPosition(q_pick_hover); % Lift the object back to hover height
pause(1);

% ==========================================
% 5. EXECUTE THE PLACE
% ==========================================
disp('Moving to destination...');
setPosition(q_place_hover); % Transit at safe height
pause(2);
setPosition(q_place_grasp); % Lower to the basket/surface
pause(1);

disp('Action: Releasing object...');
positionJaw(33); % Release 
pause(1);

setPosition(q_place_hover); % Retract to clear the object
disp('Task Completed.');