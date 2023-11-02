function botMove(mole,modelUR3,modelKUKA,Wack,Shield)

            global reset

            UR3qCurrent=modelUR3.model.getpos()
            KAKUqCurrent=modelKUKA.model.getpos()

            UR3Destination=transl([mole.molePos(Wack,:)])*trotx(pi)

            UR3q2=modelUR3.model.ikcon(UR3Destination);

            UR3q1=UR3q2;
            
            UR3q1(4)=UR3q1(4)+deg2rad(30);

            UR3q0=[0.2427	4.4808	4.6426	-1.294985	-4.71235274267090	-1.30105403435293	5.95508023803729e-06	5.95508307285278e-06	-0.0270717610752961]; % Description

            

            %hovers over mole
            UR3qmatrix = jtraj(UR3qCurrent,UR3q1,100);          
            for robotStepIndex = 1:size(UR3qmatrix,1)
                if reset==true
                    break;
                end
                modelUR3.model.animate(UR3qmatrix(robotStepIndex,:));
                drawnow;
            end
          
            %GoDown
            UR3qmatrix = jtraj(UR3q1,UR3q2,30);          
            for robotStepIndex = 1:size(UR3qmatrix,1)
                if reset==true
                    break;
                end
                modelUR3.model.animate(UR3qmatrix(robotStepIndex,:));
                drawnow;

                if robotStepIndex>25
                    if reset==true
                        break;
                    end
                    tr=transl(0,0,-(0.003*(robotStepIndex-15)*(1/15)));
                    molVertices =get(mole.moleID(Wack),'Vertices');
    
                    transformedMole = [molVertices,ones(size(molVertices,1),1)]*tr';
                    set(mole.moleID(Wack),'Vertices',transformedMole(:,1:3))   
                    
                end
            end


            %MoveBack
            UR3qmatrix = jtraj(UR3q2,UR3q0,30);          
            for robotStepIndex = 1:size(UR3qmatrix,1)
                if reset==true
                    break;
                end
                modelUR3.model.animate(UR3qmatrix(robotStepIndex,:));
                drawnow;
            end

            reset=false
end