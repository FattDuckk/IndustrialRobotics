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
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(5) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(7) = Link('d',0.11,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
            link(8) = Link('d',0.076466,'a',0,'alpha',0,'qlim',deg2rad([-0.001,0.001]), 'offset', 0);


%             link(1) = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(2) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(3) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(5) = Link('d',0.4,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(6) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(7) = Link('d',0.126,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
