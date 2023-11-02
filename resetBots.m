function resetBots(modelUR3,KUKA,UR3q0)

    if modelUR3~=0
        qCurrent=modelUR3.model.getpos();
        qmatrix = jtraj(qCurrent,UR3q0,100);          
        for robotStepIndex = 1:size(qmatrix,1)
            modelUR3.model.animate(qmatrix(robotStepIndex,:));
            drawnow;
        end
    end

end