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
% 
% q_known = [-0.607317420000000	-0.153246889642110	-1.44513262065131	0.0842857893031610	-0.0306493779284210	1.62442959657698	0.459740668926331]
% 
% T_known = robot.model.fkine(q_known);
% [q_result, error] = robot.model.ikcon(T_known, q_known);
% difference = norm(q_known - q_result);
% 
% 
% robot.model.animate(q_known);
% pause(2); % Pause for 2 seconds
% robot.model.animate(q_result);