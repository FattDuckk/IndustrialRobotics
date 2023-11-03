function [Wack,Shield,Winner,mole] = botMove(mole,modelUR3,modelKUKA,Wack,Shield,UR3q0,KUKAq0,UR3Label,KUKALabel)

            global reset

            wacking=false;
            Winner=0;
            collisionMsg = 'Collision Detected: Operation Stopped';
%% UR3
            UR3qCurrent=modelUR3.model.getpos();

%%          


            UR3Destination=transl([mole.molePos(Wack,:)])*trotx(pi);

            UR3q2=modelUR3.model.ikcon(UR3Destination);

            UR3q1=UR3q2;
            
            UR3q1(4)=UR3q1(4)+deg2rad(30);

            UR3Label.Text=["UR3 Final Joint State", num2str(UR3q2(1:7))];

            if (Wack==Shield)
                UR3q2(4)=UR3q2(4)+deg2rad(8);
                Winner=2;
            end

            
            

%% KUKA        
            KUKAqCurrent=modelKUKA.model.getpos();

            KUKADestination1=transl([mole.molePos(Shield,1:2),0.59])*trotx(pi);

            KUKADestination2=transl([mole.molePos(Shield,1:2),0.55])*trotx(pi);

            KUKAq1=modelKUKA.model.ikcon(KUKADestination1);

            KUKAq2=modelKUKA.model.ikcon(KUKADestination2);

            KUKALabel.Text=["KUKA Final Joint State", num2str([KUKAq2(1),KUKAq2(3:9)])];
            
            %%


%%

            %hovers over mole
            UR3qmatrix = jtraj(UR3qCurrent,UR3q1,100);
            KUKAqmatrix = jtraj(KUKAqCurrent,KUKAq1,100);
            [EptsUR3,EptsKUKA] = HitboxEllipsoid(modelUR3,modelKUKA);
            for robotStepIndex = 1:size(UR3qmatrix,1)
                iscollision = collisionCheck(modelUR3,modelKUKA,EptsUR3,EptsKUKA);

                if reset==true || iscollision == 1
                    if iscollision == 1
                        disp(collisionMsg)
                    end
                    break;
                end
                modelUR3.model.animate(UR3qmatrix(robotStepIndex,:));
                modelKUKA.model.animate(KUKAqmatrix(robotStepIndex,:));
                drawnow;
            end
          
            %GoDown
            UR3qmatrix = jtraj(UR3q1,UR3q2,30);   
            KUKAqmatrix = jtraj(KUKAq1,KUKAq2,15);

            
            %Check if hitting mole
            if (mole.Status(Wack)==1) && (Wack~=Shield)
                    zero_indices= find(mole.Status==0);
                    
                    if ~isempty(zero_indices)
                        random_index = datasample(zero_indices, 1);
                        mole.Status(random_index) = mole.Status(random_index) + 1;
                    end
                    mole.Status(Wack)=0;
                    wacking=true;
                    Winner=1;
            end

            %Check if hitting Shield
%             if (Wack==Shield)
%                 UR3qCurrent=modelUR3.model.getpos();
%                 UR3q2(4)=UR3q2(4)+deg2rad(7);
%                 UR3qmatrixTemp = jtraj(UR3qCurrent,UR3q2,5);
%                 UR3qmatrix(26:30,:)=UR3qmatrixTemp(1:5,:);
%                 Winner=2;
%             end
            
            %Pound the Mole
            for robotStepIndex = 1:size(UR3qmatrix,1)
                iscollision = collisionCheck(UR3qCurrent,KAKUqCurrent,modelUR3,modelKUKA);                if reset==true || iscollision == 1
                    break;
                end

                try modelUR3.model.animate(UR3qmatrix(robotStepIndex,:)); end
                try modelKUKA.model.animate(KUKAqmatrix(robotStepIndex,:)); end
                drawnow;
                
                

                if (robotStepIndex>25 && wacking==true)
                    molVertices =get(mole.moleID(Wack),'Vertices');
                    molVertices2 =get(mole.moleID(random_index),'Vertices');
                    if reset==true
                        break;
                    end
                    tr=transl(0,0,-(0.003*(robotStepIndex-15)*(1/15)));                      
                    transformedMole = [molVertices,ones(size(molVertices,1),1)]*tr';
                    set(mole.moleID(Wack),'Vertices',transformedMole(:,1:3))   


                    tr2=transl(0,0,(0.003*(robotStepIndex-15)*(1/15)));                                
                    transformedMole2 = [molVertices2,ones(size(molVertices2,1),1)]*tr2';
                    set(mole.moleID(random_index),'Vertices',transformedMole2(:,1:3)) 
                        
                    
                    
                end
            end

            pause(3)

            %MoveBack
            UR3qmatrix = jtraj(UR3q2,UR3q0,20); 
            KUKAqmatrix = jtraj(KUKAq2,KUKAq0,40);
            for robotStepIndex = 1:size(KUKAqmatrix,1)
                if reset==true
                end
            end

            UR3qmatrix = jtraj(UR3q2,UR3q0,30); 
            KUKAqmatrix = jtraj(KUKAq2,KUKAq0,30);
            for robotStepIndex = 1:size(UR3qmatrix,1)
                iscollision = collisionCheck(UR3qCurrent,KAKUqCurrent,modelUR3,modelKUKA);
                if reset==true || iscollision == 1

                    break;
                end
                try modelUR3.model.animate(UR3qmatrix(robotStepIndex,:)); end
                modelKUKA.model.animate(KUKAqmatrix(robotStepIndex,:));
                drawnow;
            end
            

            wacking=false;
            Wack=0;
            Shield=0;
            reset=false


end