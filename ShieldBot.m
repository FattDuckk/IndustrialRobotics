function ShieldBot(modelKUKA)
            iden=eye(4);

            b=iden*transl(0.143, 0, 0.586)*trotx(pi);


            KUKAq2=modelKUKA.model.ikcon(b);


            KUKAq1=KUKAq2


            KUKAq1(4)=KUKAq1(4)+deg2rad(30);

%             KUKAq0=modelKUKA.model.getpos()

            qmatrix = jtraj(KUKAq1,KUKAq2,100);          
            for robotStepIndex = 1:size(qmatrix,1)
                modelKUKA.model.animate(qmatrix(robotStepIndex,:));
                drawnow;
            end               
end