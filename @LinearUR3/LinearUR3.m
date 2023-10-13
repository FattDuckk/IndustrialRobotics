classdef LinearUR3 < RobotBaseClass
    %% LinearUR3: UR3 on a non-standard linear rail
    properties(Access = public)
        plyFileNameStem = 'LinearUR3';
        % Add any additional properties related to linear rails
    end

    methods
        %% Constructor - DEFINE ROBOT FUNCTION
        function self = LinearUR3(baseTr, useTool, toolFilename)
            % Initialize the UR3 model
            self.CreateModel();

            % Initialize linear rail parameters (if any)
            % ...

            % Set the base transform
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);

            % Initialize the tool (if any)
            if nargin >= 2 && useTool
                % Add code to attach the tool, using the toolFilename
            end

            self.PlotAndColourRobot();
        end

        %% Create the UR3 robot model
        function CreateModel(self)
            % Create the UR3 model on a linear rail
             L(1) = Link([pi     0         0         pi/2    1]); % PRISMATIC Link
              L(2) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L(3) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L(4) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
               
            %Incorporate joint limits
            L(1).qlim = [-0.8 -0.01];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-90 90]*pi/180;
            L(4).qlim = [-90 90]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;
        
            L(3).offset = -pi/2;
            L(5).offset = -pi/2;

            self.model = SerialLink(L,'name',self.name);
        end

        %% Add methods related to linear rail movement
        % You may need to add methods that handle the robot's movement
        % along the linear rail, similar to what's in your UR5 class
        % ...

    end
end


