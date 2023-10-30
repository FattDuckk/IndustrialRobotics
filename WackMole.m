function WackMole(molePos,modelUR3,Mole)
  height=(0.574)
            mole=[-0.06    0.06     height
                    0      0.06     height
                    0.06    0.06     height
                  -0.06     0       height
                   0       0       height
                   0.06     0       height
                  -0.06   -0.06     height
                   0     -0.06      height
                   0.06   -0.06      height]

            iden=eye(4);
            b=iden*transl(mole(molePos,:))*trotx(pi);

            UR3q2=modelUR3.model.ikcon(b);

            UR3q1=UR3q2;
            
            UR3q1(4)=UR3q1(4)+deg2rad(30);

            UR3q0=[0.2427	4.4808	4.6426	-1.294985	-4.71235274267090	-1.30105403435293	5.95508023803729e-06	5.95508307285278e-06	-0.0270717610752961]; % Description

            qmatrix = jtraj(UR3q0,UR3q1,100);          
            for robotStepIndex = 1:size(qmatrix,1)
                modelUR3.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end



%             for i=1:15
%                 tr=transl(0,0,-(0.01*i*(1/15)));
%                 molVertices =get(Mole,'Vertices');
% 
%                 transformedMole = [molVertices,ones(size(molVertices,1),1)]*tr';
%                 set(Mole,'Vertices',transformedMole(:,1:3))   
%                 pause(1)
%             end


            

%             qmatrix = jtraj(UR3q1,UR3q2,30);          
%             for robotStepIndex = 1:size(qmatrix,1)
% 
%                 modelUR3.model.animate(qmatrix(robotStepIndex,:));
%                 drawnow;
% 
%                 if robotStepIndex>25
%                     tr=transl(0,0,-(0.003*(robotStepIndex-15)*(1/15)));
%                     molVertices =get(Mole,'Vertices');
%     
%                     transformedMole = [molVertices,ones(size(molVertices,1),1)]*tr';
%                     set(Mole,'Vertices',transformedMole(:,1:3))   
%                     
%                 end
% 
% 
%             end

%             qmatrix = jtraj(UR3q2,UR3q0,30);          
%             for robotStepIndex = 1:size(qmatrix,1)
% 
%                 modelUR3.model.animate(qmatrix(robotStepIndex,:));
%                 drawnow;
%             end
% end