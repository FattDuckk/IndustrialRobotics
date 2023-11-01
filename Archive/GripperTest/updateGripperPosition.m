function updateGripperPosition(robot, gripper, q_current)
    endEffectorTr = robot.model.fkine(q_current);
    gripper.leftFinger.model.base = endEffectorTr;
    gripper.rightFinger.model.base = endEffectorTr;
    disp('Gripper updated to: ');
    disp(endEffectorTr);
end