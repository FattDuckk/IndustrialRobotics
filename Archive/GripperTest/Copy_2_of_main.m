%% Main Script: main.m

% Initialization
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

% Initialize the robot and gripper
[robot, gripper] = initializeRobotAndGripper();
% Display initial positions
displayInitialPositions(robot, gripper);

% Add bricks or any other elements to the environment
bricks = initializeEnvironment();

% Execute the main task for each brick
for brickIdx = 1:9
    currentVertices = get(bricks(brickIdx), 'Vertices');
    [robot, gripper] = executeTask(robot, gripper, bricks(brickIdx), get(bricks(brickIdx), 'Vertices'), currentVertices, brickIdx);
end

%% Functions

function [robot, gripper] = initializeRobotAndGripper()
    % Create instances of the LinearUR3 and CompleteGripper
    robot = LinearUR3;
    hold on;
    gripper = CompleteGripper;
    robot.model.base = transl(0, -0.3, 0.5) * trotx(pi/2); %Translate the base of the LinearUR3
    % Attach the gripper's base to the robot's end effector
    endEffectorTr = robot.model.fkine(zeros(1,7));
    gripper.leftFinger.model.base = endEffectorTr;
    gripper.rightFinger.model.base = endEffectorTr;
end

function [bricks, initialVertices] = initializeEnvironment()
    % Set up any bricks, safety zones, walls, etc. for the environment
    % Place the table
    tableName = 'tableBrown2.1x1.4x0.5m.ply';
    tableLocation = [0, 0, 0]; % Adjust position as needed
    PlaceObject(tableName, tableLocation);
    
   hold on;
    bricks = gobjects(1, 9);
    for brickIdx = 1:9
        bricks(brickIdx) = PlaceObject('brick.ply', [0, 0, 0]);
        brickTransform = transl(-0.45, 0.5 + -0.2 * brickIdx, 0.5);  % Adjust as necessary
        initialVertices = get(bricks(brickIdx), 'Vertices');
        transformedVertices = (brickTransform * [initialVertices'; ones(1, size(initialVertices, 1))])';
        set(bricks(brickIdx), 'Vertices', transformedVertices(:, 1:3));
    end
end

function displayInitialPositions(robot, gripper)
    % Display the initial positions of the robot and gripper
    robot.model.animate(zeros(1,7));  % Assuming 7 joints
    gripper.leftFinger.model.animate([0, 0.2]); 
    gripper.rightFinger.model.animate([0, -0.2]);
    % Set the axis limits after displaying the robot and gripper
    plotWorkspace = [-1 1 -1 1 0 1.5];
    axis(plotWorkspace);
    pause(5);
end

function [robot, gripper, currentVertices] = executeTask(robot, gripper, brick, initialVertices, currentVertices,brickIdx)
  
    
    % Gripper Offset Adjustment
    gripper_offset = 0.47; % in meters or -260mm
 P_pickup_original = [-0.45, 0.5 + 0.1 * brickIdx, 0.45];
    P_dropoff_original = [0.5, 0.5 + 0.1 * brickIdx, 0.45];
    P_pickup = P_pickup_original + [0, 0, gripper_offset];
    P_dropoff = P_dropoff_original + [0, 0, gripper_offset];
    disp(['Intended Pickup Position: ', num2str(P_pickup)]);


    % Approach orientation: vertically downwards
    approach_orientation = trotx(pi)* trotz(pi/2); 

    % Inverse kinematics
    q_current = [-0.607317420000000	-0.153246889642110	-1.44513262065131	0.0842857893031610	-0.0306493779284210	1.62442959657698	0.459740668926331];
    q_dropoff_guess = [-0.7769    0.0306    1.3562    0.0230    0.0919   -1.8696         0];
    q_pickup = robot.model.ikcon((transl(P_pickup) * approach_orientation), q_current)';  % Use q_current as the initial guess
    q_dropoff = robot.model.ikcon((transl(P_dropoff) * approach_orientation), q_dropoff_guess)';
    achieved_pickup_position = robot.model.fkine(q_pickup).t;
    disp(['Achieved Pickup Position by IK: ', num2str(achieved_pickup_position')]);

    % Gripper finger positions
    q_gripper_open = [0, 0.6];
    q_gripper_closed = [0, -0.2];
    brickPickedUp = false;

    % Animation steps
    nSteps = 200;
    q_traj = jtraj(q_pickup, q_dropoff, nSteps);
    prevEndEffectorPos = robot.model.fkine(q_pickup).t(1:3);  % Store the initial position

    for step = 1:nSteps
        % Animate robot and gripper
        robot.model.animate(q_traj(step, :));
        endEffectorTr = robot.model.fkine(q_traj(step, :));
       
        gripper.leftFinger.model.base = endEffectorTr;
        gripper.rightFinger.model.base = endEffectorTr;
        
        gripper.leftFinger.model.animate(gripper.leftFinger.model.getpos());
        gripper.rightFinger.model.animate(gripper.rightFinger.model.getpos());

        currentPos = robot.model.fkine(robot.model.getpos()).t;

        distanceToPickup = norm(currentPos(1:3) - (P_pickup' + [gripper_offset; 0; 0]));
        disp(['Distance to Pickup: ', num2str(distanceToPickup)]);
        disp(['Brick Picked Up: ', num2str(brickPickedUp)]);

        if abs(currentPos(1) - P_pickup(1)) < 0.1 && abs(currentPos(2) - P_pickup(2)) < 0.15 && abs(currentPos(3) - P_pickup(3)) < 0.15 && ~brickPickedUp
    
            disp('Attempting to pick up the brick...');
            disp(['Current Pos: ', num2str(currentPos'), ' Desired Pos: ', num2str(P_pickup)]);
            gripper.leftFinger.model.animate(q_gripper_closed);
            gripper.rightFinger.model.animate(-q_gripper_closed);
            brickPickedUp = true;
            % brickCenter = mean(get(brick, 'Vertices')); % Compute the center of the brick
            % initialOffset = brickCenter' - currentPos;  % Calculate the offset
        
        elseif abs(currentPos(1) - P_dropoff(1)) < 0.06 && abs(currentPos(2) - P_dropoff(2)) < 0.15 && abs(currentPos(3) - P_dropoff(3)) < 0.05
    
            disp('Attempting to drop the brick...');
            disp(['Current Pos: ', num2str(currentPos'), ' Desired Pos: ', num2str(P_dropoff)]);
            gripper.leftFinger.model.animate(q_gripper_open);
            gripper.rightFinger.model.animate(-q_gripper_open);
            brickPickedUp = false;
        end
  if brickPickedUp
        tr_obj = robot.model.fkine(robot.model.getpos());
        
        z_offset = 0.3; % 300mm in the z-axis
        offsetTransform = transl(0, 0, z_offset);
        x_rotation = trotx(-0.1);

        z_rotation = trotz(pi/2); % Rotate by 90 degrees about the z-axis
        
        tr = tr_obj.T * offsetTransform *x_rotation* z_rotation;  % Combine transformations
        transformedVertices = (tr * [initialVertices'; ones(1, size(initialVertices, 1))])';
        
        set(brick, 'Vertices', transformedVertices(:, 1:3));
        drawnow();
    end
        plotWorkspace = [-1 1 -1 1 0 1.5];
            axis(plotWorkspace);
            drawnow();
            pause(0.05);
        end
end