function resetBots(modelUR3,KUKA,UR3q0,KUKAq0)

    if modelUR3==0
        qKUKACurrent=KUKA.model.getpos();
        qKUKAmatrix = jtraj(qKUKACurrent,KUKAq0,100);

        for robotStepIndex = 1:size(qKUKAmatrix,1)
            KUKA.model.animate(qKUKAmatrix(robotStepIndex,:));
            drawnow;
        end
    


    elseif KUKA==0
        qUR3Current=modelUR3.model.getpos();
        qUR3matrix = jtraj(qUR3Current,UR3q0,100);

        for robotStepIndex = 1:size(qUR3matrix,1)
            modelUR3.model.animate(qUR3matrix(robotStepIndex,:));
            modelUR3.model.animate(qUR3matrix(robotStepIndex,:));
            drawnow;
        end

    else
        qUR3Current=modelUR3.model.getpos();
        qKUKACurrent=KUKA.model.getpos();

        qUR3matrix = jtraj(qUR3Current,UR3q0,100);
        qKUKAmatrix = jtraj(qKUKACurrent,KUKAq0,100);
        for robotStepIndex = 1:size(qUR3matrix,1)
            modelUR3.model.animate(qUR3matrix(robotStepIndex,:));
            KUKA.model.animate(qKUKAmatrix(robotStepIndex,:));
            drawnow;
        end
    end

end