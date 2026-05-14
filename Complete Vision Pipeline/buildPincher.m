function robot = buildPincher()
    % Creates a rigidBodyTree digital twin of the Phantom X Pincher
    
    % Initialize the tree
    robot = rigidBodyTree('DataFormat', 'row');

    % Define your DH Parameters from Task 5.5
    a     = [0,      105,  106,  75];
    alpha = [pi/2,   0,     0,     0];
    d     = [145,   0,     0,     0];

    % Build the kinematic chain
    parentName = 'base';

    for i = 1:4
        bodyName = sprintf('link%d', i);
        jointName = sprintf('joint%d', i);
        
        body = rigidBody(bodyName);
        joint = rigidBodyJoint(jointName, 'revolute');
        
        % MATLAB 'dh' order is: [a, alpha, d, theta]
        setFixedTransform(joint, [a(i), alpha(i), d(i), 0], 'dh');
        
        body.Joint = joint;
        addBody(robot, body, parentName);
        
        parentName = bodyName; % Current link becomes parent for the next one
    end
end