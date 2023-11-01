%% Main Script: main.m

% Initialization
clf;
clear;
clc;

% Define the workspace dimensions for plotting
plotWorkspace = [-1 1 -1 1 0 1.5];
axis(plotWorkspace);
view(3);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% Initialize the robot, gripper, and environment
[robot, gripper] = initializeRobotAndGripper();

% Add bricks or any other elements to the environment
initializeEnvironment();

% Display initial positions
displayInitialPositions(robot, gripper);

% Safety checks
if ~safetyCheck(robot, gripper)
    error('Safety limits breached during initialization! Stopping simulation.');
end

% Execute the main task
[robot, gripper] = executeTask(robot, gripper);

%% Functions

function [robot, gripper] = initializeRobotAndGripper()
    % Create instances of the LinearUR3 and CompleteGripper
    robot = LinearUR3;
    gripper = CompleteGripper;

    % Attach the gripper's base to the robot's end effector
    endEffectorTr = robot.model.fkine(zeros(1,7));
    gripper.leftFinger.model.base = endEffectorTr;
    gripper.rightFinger.model.base = endEffectorTr;
end

function initializeEnvironment()
    % Set up any bricks, safety zones, walls, etc. for the environment
    % Add your initialization code here...
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

function isSafe = safetyCheck(robot, gripper)
    % Check if the robot and gripper are within the safety limits
    % Add your safety checks here...
    
    isSafe = true; % Placeholder, you should replace this with your logic
end


function [robot, gripper] = executeTask(robot,gripper)
 

    % Define the pickup and drop-off positions
    P_pickup = [-0.7, 0.5, 0.2];
    P_dropoff = [0.1, 0.5, 0.5];

    % Approach orientation: vertically downwards
    approach_orientation = trotx(pi); 

    % Inverse kinematics
    q_pickup = robot.model.ikcon(transl(P_pickup) * approach_orientation)';
    q_dropoff = robot.model.ikcon(transl(P_dropoff) * approach_orientation)';

    % Gripper finger positions
    q_gripper_open = [0, 0.2];
    q_gripper_closed = [0, -0.2];

    % Animation steps
    nSteps = 50;
    q_traj = jtraj(q_pickup, q_dropoff, nSteps);

    for step = 1:nSteps
        % Animate robot and gripper
        robot.model.animate(q_traj(step, :));
        endEffectorTr = robot.model.fkine(q_traj(step, :));
        gripper.leftFinger.model.base = endEffectorTr;
        gripper.rightFinger.model.base = endEffectorTr;
        
        gripper.leftFinger.model.animate(gripper.leftFinger.model.getpos());
        gripper.rightFinger.model.animate(gripper.rightFinger.model.getpos());

        % Adjust gripper fingers based on robot's position
        currentPos = robot.model.fkine(robot.model.getpos()).t;
        if norm(currentPos(1:3) - P_pickup') < 0.05
            gripper.leftFinger.model.animate(q_gripper_closed);
            gripper.rightFinger.model.animate(-q_gripper_closed);
        elseif norm(currentPos(1:3) - P_dropoff') < 0.05
            gripper.leftFinger.model.animate(q_gripper_open);
            gripper.rightFinger.model.animate(-q_gripper_open);
        end
 % Set the axis limits
        plotWorkspace = [-1 1 -1 1 0 1.5];
        axis(plotWorkspace);
        drawnow();
        pause(0.05);
    end
end
