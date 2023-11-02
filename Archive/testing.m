            link(1) = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(5) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(7) = Link('d',0.126,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);


  robot= SerialLink(link)       

  q=zeros(1,7)
  robot.plot(q,'workspace',[-2 2 -2 2 -0.05 2],'scale',1);

  %%

%   robot=KUKA
r=UR3
axis equal
  r.model.teach()
% robot.TestMoveJoints


a=T(r.model.fkine(r.model.getpos()))

%%
robot=KUKA
r=UR3
qrKUKA = [0	pi/3   0	    5*pi/12	    0	    pi/4   pi 0 0];
robot.model.teach(qrKUKA)


%%

qrKUKA = [0	pi/3   0	    pi/4	    0	    5*pi/12   pi 0 0];
        qrUR3 = [0	-3*pi/4   -pi/4	-pi/2           pi/2	-pi/2   0 0];

height=0.7
%%

robot=KUKA
qrKUKA = [0	pi/3   0	    pi/4	    0	    5*pi/12   pi 0 0];
pos=robot.model.fkine(qrKUKA)

r=UR3
qrUR3 = [0	-3*pi/4   -pi/4	-pi/2           pi/2	-pi/2   0 0];
pos2=r.model.fkine(qrUR3)

height=0.7
coordinate=[-0.06  0.06    height
         -0.06  0       height
         -0.06  -0.6    height
            0   0.06    height
            0   0       height
            0   -0.6    height
          0.06 0.06     height
          0.06 0        height
          0.06 -0.6     height]