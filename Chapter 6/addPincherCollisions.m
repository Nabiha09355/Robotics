function robot = addPincherCollisions(robot)
    % Adds collision geometries to the Phantom X Pincher rigidBodyTree
    
    % Link lengths from Chapter 5 (in mm)
    d1 = 145; 
    a2 = 105; 
    a3 = 106; 
    a4 = 75;
    
    radius = 18; % Visual thickness of the robot links
    
    % 1. Link 1 (Base to Shoulder)
    % Frame {1} is at the shoulder. Due to alpha = pi/2 twist, its -Y axis 
    % points straight down to the base. We rotate the Z-aligned cylinder 
    % to Y, and shift it halfway down.
    coll1 = collisionCylinder(radius, d1);
    T1 = trvec2tform([0, -d1/2, 0]) * axang2tform([1, 0, 0, pi/2]);
    addCollision(robot.Bodies{1}, coll1, T1);

    % 2. Link 2 (Shoulder to Elbow)
    % Frame {2} is at the elbow. Its -X axis points backward to the shoulder.
    % We rotate the Z-aligned cylinder to X, and shift it backward.
    coll2 = collisionCylinder(radius, a2);
    T2 = trvec2tform([-a2/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{2}, coll2, T2);

    % 3. Link 3 (Elbow to Wrist)
    % Frame {3} is at the wrist. -X points backward to the elbow.
    coll3 = collisionCylinder(radius, a3);
    T3 = trvec2tform([-a3/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{3}, coll3, T3);

    % 4. Link 4 (Wrist to Gripper)
    % Frame {4} is at the end-effector. -X points backward to the wrist.
    coll4 = collisionCylinder(radius, a4);
    T4 = trvec2tform([-a4/2, 0, 0]) * axang2tform([0, 1, 0, pi/2]);
    addCollision(robot.Bodies{4}, coll4, T4);
    
    disp('Collision geometries successfully added and aligned.');
end