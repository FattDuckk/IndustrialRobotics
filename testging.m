link(1) = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(2) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(3) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(5) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(6) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(7) = Link('d',0.126,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
robot=SerialLink(link)
q=[0 0 0 0 0 0 0]
robot.teach()