clf
clc
clear


mesh_h = PlaceObject('brick.ply');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,0.1)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));
transformedVertices = [vertices,ones(size(vertices,1),1)] * trotx(pi/2)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));
mdl_planar3
hold on
p3.plot([0,0,0])
p3.delay = 0;

axis([-3,3,-3,3,-0.5,8])

for i = -pi/4:0.01:pi/4
    p3.animate([i,i,i])
    tr = p3.fkine([i,i,i]);
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    set(mesh_h,'Vertices',transformedVertices(:,1:3));
    drawnow();
    pause(0.01);
end

%%
%% Initialization
clf;
clear;
clc;
hold on;

% Define the workspace dimensions for plotting
plotWorkspace = [-1 1 -1 1 -1 1.5];
axis(plotWorkspace);
view(3);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% Place the table and brick
tableName = 'tableBrown2.1x1.4x0.5m.ply';
tableLocation = [0, 0, 0];
PlaceObject(tableName, tableLocation);
brick = PlaceObject('brick.ply', [-0.55, 0.5, 0.54]);
initialVertices = get(brick, 'Vertices');

% Initialize robot and gripper
robot = LinearUR3();
gripper = CompleteGripper();
robot.model.base = transl(0, -0.3, 0.5) * trotx(pi/2);
% Move robot to pick up position
P_pickup = [-0.55, 0.5, 1.2];
approach_orientation = trotx(pi) * trotz(pi/2); 
q_pickup = robot.model.ikcon(transl(P_pickup) * approach_orientation);
robot.model.animate(q_pickup);

% Update gripper's position
endEffectorTr = robot.model.fkine(q_pickup);
gripper.leftFinger.model.base = endEffectorTr;
gripper.rightFinger.model.base = endEffectorTr;
% Simulate grasping the brick
brickOffset = [0, 0, -0.2];  % Adjust as needed
newBrickPos = P_pickup + brickOffset;
set(brick, 'Vertices', initialVertices + repmat(newBrickPos, size(initialVertices, 1), 1));

% Move robot to drop off position
P_dropoff = [0.5, 0.5, 1.2];
q_dropoff = robot.model.ikcon(transl(P_dropoff) * approach_orientation);
q_traj = jtraj(q_pickup, q_dropoff, 100);

for q = q_traj'
    robot.model.animate(q);
    endEffectorPos = robot.model.fkine(q).t(1:3);
    deltaMove = endEffectorPos - P_pickup;
    newBrickPos = [-0.55, 0.5, 0.54] + deltaMove' + brickOffset;
    set(brick, 'Vertices', initialVertices + repmat(newBrickPos, size(initialVertices, 1), 1));
    drawnow();
    pause(0.05);
end


%%
%% Initialization
clf;
clear;
clc;

% Define the workspace dimensions for plotting
plotWorkspace = [-1 1 -1 1 -1 1.5];
axis(plotWorkspace);
view(3);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% Initialize the environment: Add table and brick
[brick] = initializeEnvironment();

% Initialize the robot and gripper
[robot, gripper] = initializeRobotAndGripper();

% Display initial positions and run teach
displayAndTeach(robot, gripper);



function [robot, gripper] = initializeRobotAndGripper()
    % Create instances of the LinearUR3 and CompleteGripper
    robot = LinearUR3;
    gripper = CompleteGripper;
    
    % Translate the base of the LinearUR3
    robot.model.base = transl(0, -0.3, 0.5) * trotx(pi/2); 
    
    % Attach the gripper's base to the robot's end effector
    endEffectorTr = robot.model.fkine(zeros(1,7));
    gripper.leftFinger.model.base = endEffectorTr;
    gripper.rightFinger.model.base = endEffectorTr;
end

function [brick] = initializeEnvironment()
    % Set up any bricks, safety zones, walls, etc. for the environment
    
    % Place the table
    tableName = 'tableBrown2.1x1.4x0.5m.ply';
    tableLocation = [0, 0, 0]; 
    PlaceObject(tableName, tableLocation);
    
    hold on
    
    % Place the brick
    brick = PlaceObject('brick.ply', [-0.55, 0.5, 0.54]);
end

function displayAndTeach(robot, gripper)
    % Display the initial positions of the robot and gripper
    robot.model.animate(zeros(1,7));  % Assuming 7 joints
    gripper.leftFinger.model.animate([0, 0.2]);
    gripper.rightFinger.model.animate([0, -0.2]);
    
    % Set the axis limits after displaying the robot and gripper
    plotWorkspace = [-1 1 -1 1 0 1.5];
    axis(plotWorkspace);
    q = (zeros(1,7));
    % Run the teach function
    robot.model.teach(q);
    
end

%%

% -0.607317420000000	-0.153246889642110	-1.44513262065131	0.0842857893031610	-0.0306493779284210	1.62442959657698	0.459740668926331
%%




% Clearing and setting up the environment

clc
clear

% Loading the UR3 and gripper models
ur3 = LinearUR3;
gripper = CompleteGripper;

% Initial position for UR3 for visualization
initial_angles = [0, 0, 0, 0, 0, 0, 0]; 
ur3.model.animate(initial_angles);

% Attach the gripper to the end effector of UR3
end_effector_pose = ur3.model.fkine(initial_angles);
gripper.leftFinger.model.base = end_effector_pose;
gripper.rightFinger.model.base = end_effector_pose;

% Animate the gripper's initial state (open position)
initial_gripper_angles = [0, 0];  % Assuming the gripper starts in an open position
gripper.leftFinger.model.animate(initial_gripper_angles);
gripper.rightFinger.model.animate(initial_gripper_angles);

% Setting up the brick visualization
mesh_h = PlaceObject('brick.ply');
axis equal
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(0, 0, 0.1)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3));
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * trotx(pi/2)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3));
hold on

% Calculate the pose for UR3 to be close to the brick
desired_pose = transl(0.1, 0.1, 0.1);
initial_angles = ur3.model.ikcon(desired_pose); 
ur3.model.animate(initial_angles);

% Adjust the gripper position based on the new end effector pose
end_effector_pose = ur3.model.fkine(initial_angles);
gripper.leftFinger.model.base = end_effector_pose;
gripper.rightFinger.model.base = end_effector_pose;

% Animate the gripper closing on the brick 
closing_angles_left = [0, -pi/8];  
closing_angles_right = [0, pi/8];  
gripper.leftFinger.model.animate(closing_angles_left);
gripper.rightFinger.model.animate(closing_angles_right);

% Attach the brick to the robot's end effector
tr = ur3.model.fkine(initial_angles);
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3));

% Move the robot to the new location
desired_new_pose = transl(0.3, 0.3, 0.1);
new_angles = ur3.model.ikcon(desired_new_pose, initial_angles);
ur3.model.animate(new_angles);

% Animate the gripper releasing the brick 
opening_angles_left = [0, 0];  
opening_angles_right = [0, 0];  
gripper.leftFinger.model.animate(opening_angles_left);
gripper.rightFinger.model.animate(opening_angles_right);



%%
 2.4 Sample the joint angles within the joint limits at 30 degree increments
between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that
you end up with a big list of points
stepRads = deg2rad(30);
qlim = densoRobot.qlim;
% Don't need to worry about joint 6
pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic
for q1 = qlim(1,1):stepRads:qlim(1,2)
for q2 = qlim(2,1):stepRads:qlim(2,2)
for q3 = qlim(3,1):stepRads:qlim(3,2)
for q4 = qlim(4,1):stepRads:qlim(4,2)
for q5 = qlim(5,1):stepRads:qlim(5,2)
% Don't need to worry about joint 6, just assume it=0
q6 = 0;
% for q6 = qlim(6,1):stepRads:qlim(6,2)
q = [q1,q2,q3,q4,q5,q6];
tr = densoRobot.fkine(q).T;
pointCloud(counter,:) = tr(1:3,4)';
counter = counter + 1;
if mod(counter/pointCloudeSize * 100,1) == 0
display(['After ',num2str(toc),' seconds, completed
',num2str(counter/pointCloudeSize * 100),'% of poses']);
end
% end
end
end
end
end
end

% 2.6 Create a 3D model showing where the end effector can be over all these
samples.
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.')


