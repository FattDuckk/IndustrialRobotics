
clear
close all         

% Make, save and plot some robots      
        qrKUKA = [-0.02 0 0	pi/3   0	    pi/4	    0	    5*pi/12   pi 0 0];
       UR3q0 = [0.2427	4.3808	4.6426	-1.194985	-4.71235274267090	-1.30105403435293	5.95508023803729e-06	5.95508307285278e-06	-0.0270717610752961]; % Description

hold on
            modelUR3 = ModifiedUR3;
            modelUR3.model.base = transl([-0.6,0,0.574]);
            modelUR3.model.animate(UR3q0);
            drawnow();


            modelKUKA = KUKA;
            modelKUKA.model.base = transl([0.8,0,0.2]);
            modelKUKA.model.animate(qrKUKA);
            drawnow();

            position=modelUR3.model.fkine(UR3q0)
%             Mole=PlaceObject('Base2.ply', [0,0,0.5]);
% 
% 
%             for i=1:15
%                 tr=transl(0,0,-(0.01*i*(1/15)));
%                 molVertices =get(Mole,'Vertices');
% 
%                 transformedMole = [molVertices,ones(size(molVertices,1),1)]*tr';
%                 set(Mole,'Vertices',transformedMole(:,1:3))   
%                 pause(1)
%             end
PlaceObject('Base2.ply', [0,0,0.5]);

            campos([-0.0045  -10.5324    6.3959])
            camtarget([0.1088   -0.0471    0.7224])

            %%

            iden=eye(4);

            b=iden*transl(0.143, 0, 0.586)*trotx(pi);


            KUKAq2=[-0.2 0-2.6738    5.3639   -1.0478    5.2576    3.9018    1.5458   -3.5082   -0.0000   -0.0000]
%             modelKUKA.model.ikcon(b)


            KUKAq1=KUKAq2


            KUKAq1(4)=KUKAq1(4)+deg2rad(30);

%             KUKAq0=modelKUKA.model.getpos()

            qmatrix = jtraj(KUKAq1,KUKAq2,100);          
            for robotStepIndex = 1:size(qmatrix,1)
                modelKUKA.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end               
%%

            b=iden*trotx(pi)*transl(mole(8,:));

            UR3q2=modelUR3.model.ikcon(b);

            UR3q1=UR3q2;
            
            UR3q1(4)=UR3q1(4)+deg2rad(30);

            UR3q0=modelUR3.model.getpos()

            qmatrix = jtraj(UR3q0,UR3q1,100);          
            for robotStepIndex = 1:size(qmatrix,1)
                modelUR3.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end



            qmatrix = jtraj(UR3q1,UR3q2,30);          
            for robotStepIndex = 1:size(qmatrix,1)

                modelUR3.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end

            %%


            height=-(0.574)
            mole=[-0.06    0.06     height
                  -0.06     0       height
                  -0.06   -0.06     height
                    0      0.06     height
                    0       0       height
                    0     -0.06      height
                   0.06    0.06     height
                   0.06     0       height
                   0.06   -0.06      height]

            iden=eye(4);
            b=iden*trotx(pi)*transl(mole(8,:));

            UR3q2=modelUR3.model.ikcon(b);

            UR3q1=UR3q2;
            
            UR3q1(4)=UR3q1(4)+deg2rad(30);

            UR3q0=modelUR3.model.getpos()

            qmatrix = jtraj(UR3q0,UR3q1,100);          
            for robotStepIndex = 1:size(qmatrix,1)
                modelUR3.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end



            qmatrix = jtraj(UR3q1,UR3q2,30);          
            for robotStepIndex = 1:size(qmatrix,1)

                modelUR3.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end