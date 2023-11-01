clc
clear
clf

plotWorkspace = [-1 1 -1 1 -1 1.5];
axis(plotWorkspace);
view(3);
grid on;

% Loading the UR3 and gripper models
ur3 = LinearUR3;
gripper = CompleteGripper;

% Initial position for UR3 for visualization
initial_angles = [0, 0, 0, 0, 0, 0, 0]; 
ur3.model.animate(initial_angles);
hold on

% Setting up the brick visualization
mesh_h = PlaceObject('brick.ply');
axis equal
vertices = get(mesh_h, 'Vertices');
brickTransform = transl(0.1, 0.1, -0.2);  % Lower the initial position of the brick by 0.2 units
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * brickTransform';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3));

% Calculate the pose for UR3 to approach the brick from above
desired_pose = transl(0.1, 0.1, 0) * trotz(pi) * trotx(-pi/2);  % Adjust the Z position to 0 for the downward approach
target_angles = ur3.model.ikcon(desired_pose); 

% Generate a trajectory from the initial angles to the target angles
t = [0:0.05:2];  % Time vector, adjust as needed
q0 = initial_angles;
qf = target_angles;
[qtraj,qdtraj,qddtraj] = jtraj(q0, qf, t);
initial_gripper_angles = [0, 0];  % Assuming the gripper starts in an open position

% Animate the robot and gripper approaching the brick
for i = 1:length(t)
    ur3.model.animate(qtraj(i,:));
    end_effector_pose = ur3.model.fkine(qtraj(i,:));
    gripper.leftFinger.model.base = end_effector_pose;
    gripper.rightFinger.model.base = end_effector_pose;

    % Animate the gripper's fingers (assuming they remain in the initial position during movement)
    gripper.leftFinger.model.animate(initial_gripper_angles);
    gripper.rightFinger.model.animate(initial_gripper_angles);

    pause(0.05);
end
% Animate the gripper closing on the brick 
closing_angles_left = [0, -pi/8];  
closing_angles_right = [0, pi/8];  
gripper.leftFinger.model.animate(closing_angles_left);
gripper.rightFinger.model.animate(closing_angles_right);

% Attach the brick to the robot's end effector
tr = ur3.model.fkine(target_angles).T;
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3));

% Move the robot to the new location
desired_new_pose = transl(0.3, 0.3, 0.1);
new_angles = ur3.model.ikcon(desired_new_pose, target_angles);

% Generate a trajectory from the target angles to the new angles
q0 = target_angles;
qf = new_angles;
[qtraj,qdtraj,qddtraj] = jtraj(q0, qf, t);

% Animate the robot and gripper moving to the new position with the brick
for i = 1:length(t)
    ur3.model.animate(qtraj(i,:));
    end_effector_pose = ur3.model.fkine(qtraj(i,:));
    gripper.leftFinger.model.base = end_effector_pose;
    gripper.rightFinger.model.base = end_effector_pose;
    
    % Update the brick's position based on the end effector pose
    tr = end_effector_pose.T;
    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr';
    set(mesh_h, 'Vertices', transformedVertices(:, 1:3));

    % Animate the gripper's fingers (assuming they remain closed around the brick during movement)
    gripper.leftFinger.model.animate(closing_angles_left);
    gripper.rightFinger.model.animate(closing_angles_right);

    pause(0.05);
end

% Animate the gripper releasing the brick 
opening_angles_left = [0, 0];  
opening_angles_right = [0, 0];  
gripper.leftFinger.model.animate(opening_angles_left);
gripper.rightFinger.model.animate(opening_angles_right);
