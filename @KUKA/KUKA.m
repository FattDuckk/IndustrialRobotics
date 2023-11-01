classdef KUKA< RobotBaseClass

    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'KUKA';
    end
    
    methods
%% Constructor
        function self = KUKA(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]);
            link(2) = Link([0      0.067  0       0   0]);
            link(3) = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(5) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(7) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(8) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(9) = Link('d',0.11,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
            link(10) = Link('d',0.076466,'a',0,'alpha',0,'qlim',deg2rad([-0.001,0.001]), 'offset', 0);
            link(11) = Link('d',0.03709,'a',-0.1496,'alpha',0,'qlim',deg2rad([-0.001,0.001]), 'offset', 0);


            link(1).qlim = [-0.8 -0.01];
            link(2).offset = pi/2;
    
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
