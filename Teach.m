function Teach(q,joint,robot)

qCurrent=robot.model.getpos()
qNew=qCurrent
q=deg2rad(q)
qNew(1,joint)=q
% robot.model.animate(qNew)


qmatrix = jtraj(qCurrent,qNew,10);          
    for robotStepIndex = 1:size(qmatrix,1)
        robot.model.animate(qmatrix(robotStepIndex,:));
        drawnow;
    end       

end