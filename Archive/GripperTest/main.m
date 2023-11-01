%% Main Script: main.m

% Initialization
clf;
clear;
clc;

% Define the workspace dimensions for plotting
plotWorkspace = [-2 2 -2 2 0 1.5];
axis(plotWorkspace);
view(3);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% Add bricks or any other elements to the environment
[brick, initialVertices] = initializeEnvironment();

% Initialize the robot, gripper, and environment
[robot, gripper] = initializeRobotAndGripper();

% Display initial positions
displayInitialPositions(robot, gripper);

% Execute the main task
[robot, gripper] = executeTask(robot, gripper, brick, initialVertices);

%% Functions

function [robot, gripper] = initializeRobotAndGripper()
    % Create instances of the LinearUR3 and CompleteGripper
    robot = LinearUR3;
    gripper = CompleteGripper;
    robot.model.base = transl(0, -0.3, 0.5) * trotx(pi/2);
    endEffectorTr = robot.model.fkine(zeros(1,7));
    gripper.leftFinger.model.base = endEffectorTr;
    gripper.rightFinger.model.base = endEffectorTr;
end

function [brick, initialVertices] = initializeEnvironment()
    tableName = 'tableBrown2.1x1.4x0.5m.ply';
    tableLocation = [0, 0, 0];
    PlaceObject(tableName, tableLocation);
    
    hold on
    brick = PlaceObject('brick.ply', [-0.45, 0.5, 0.5]);
    initialVertices = get(brick, 'Vertices');
end

function displayInitialPositions(robot, gripper)
    robot.model.animate(zeros(1,7));
    gripper.leftFinger.model.animate([0, 0.2]);
    gripper.rightFinger.model.animate([0, -0.2]);
    plotWorkspace = [-2 2 -2 2 0 1.5];
    axis(plotWorkspace);
    pause(5);
end

function [robot, gripper] = executeTask(robot, gripper, brick, initialVertices)
    gripper_offset = 0.47;
    P_pickup_original = [-0.45, 0.5, 0.5];
    P_dropoff_original = [0.5, 0.5, 0.5];
    P_pickup = P_pickup_original + [0, 0, gripper_offset];
    P_dropoff = P_dropoff_original + [0, 0, gripper_offset];
    disp(['Intended Pickup Position: ', num2str(P_pickup)]);

    approach_orientation = trotx(pi)* trotz(pi/2);
    q_current = [-0.607317420000000, -0.153246889642110, -1.44513262065131, 0.0842857893031610, -0.0306493779284210, 1.62442959657698, 0.459740668926331];
    q_dropoff_guess = [-0.7769, 0.0306, 1.3562, 0.0230, 0.0919, -1.8696, 0];
    q_pickup = robot.model.ikcon((transl(P_pickup) * approach_orientation), q_current)';
    q_dropoff = robot.model.ikcon((transl(P_dropoff) * approach_orientation), q_dropoff_guess)';
    achieved_pickup_position = robot.model.fkine(q_pickup).t;
    disp(['Achieved Pickup Position by IK: ', num2str(achieved_pickup_position')]);

    q_gripper_open = [0, 0.6];
    q_gripper_closed = [0, -0.2];
    brickPickedUp = false;
    nSteps = 200;
    q_traj = jtraj(q_pickup, q_dropoff, nSteps);
    prevEndEffectorPos = robot.model.fkine(q_pickup).t(1:3);

    for step = 1:nSteps
        robot.model.animate(q_traj(step, :));
        endEffectorTr = robot.model.fkine(q_traj(step, :));
        gripper.leftFinger.model.base = endEffectorTr;
        gripper.rightFinger.model.base = endEffectorTr;
        gripper.leftFinger.model.animate(gripper.leftFinger.model.getpos());
        gripper.rightFinger.model.animate(gripper.rightFinger.model.getpos());
        currentPos = robot.model.fkine(robot.model.getpos()).t;

     if norm(currentPos(1:3) - P_pickup') < 0.1 && ~brickPickedUp
    disp('Attempting to pick up the brick...');
    brickPickedUp = true;
    gripper.leftFinger.model.animate(q_gripper_closed);
    gripper.rightFinger.model.animate(-q_gripper_closed);
    % Calculate the relative offset between the brick and the gripper
    brickOffset = mean(initialVertices, 1) - currentPos(1:3)';
end


        if norm(currentPos(1:3) - P_dropoff') < 0.1 && brickPickedUp
            disp('Attempting to drop the brick...');
            brickPickedUp = false;
            gripper.leftFinger.model.animate(q_gripper_open);
            gripper.rightFinger.model.animate(-q_gripper_open);
        end

  if brickPickedUp
    disp('Attempting to move the brick...');

    % This is the transformation matrix for the end effector
    endEffectorTransform = endEffectorTr;
    
    % Apply this transformation to the original vertices A (initialVertices)
    transformedVertices = ((endEffectorTransform.T) * [initialVertices, ones(size(initialVertices, 1), 1)]')';

    % Set these transformed vertices for visualization
    set(brick, 'Vertices', transformedVertices(:, 1:3));
end
        plotWorkspace = [-2 2 -2 2 0 1.5];
        axis(plotWorkspace);
        drawnow();
        pause(0.05);
    end
end

% function [brick, updatedVertices] = moveBrickCustom(currentPos, prevEndEffectorPos, brick, currentVertices)
%     % Calculate the movement of the end effector
%     deltaMove = currentPos - prevEndEffectorPos;
% 
%     disp(['Delta Move: ', num2str(deltaMove')]);
% 
%     % Apply this movement to the brick's position
%     transformedVertices = currentVertices + repmat(deltaMove', size(currentVertices, 1), 1);
% 
%     set(brick, 'Vertices', transformedVertices);
%     drawnow();
%     updatedVertices = transformedVertices;
