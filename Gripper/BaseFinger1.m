classdef BaseFinger1 < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'GripperUR3';  % The stem name for your ply files
    end

    methods
        %% Constructor
        function self = BaseFinger1()
            % Initialize the gripper model
            self.CreateModel();

            % Set base transform
            % if nargin < 1
            %     baseTr = eye(4);
            % end
            %self.model.base = self.model.base.T * baseTr;

            % Plot and color the robot using PLY files
            self.PlotAndColourRobot();
        end

        %% Create Gripper Model
           function CreateModel(self)
            % Define the D&H parameters
            link(1) = Link('d', 0.20143, 'a', -0.03366, 'alpha', pi/2, 'qlim', [0,0]);
            link(2) = Link('d', 0, 'a', 0, 'alpha',0, 'qlim', [-pi/4, pi/4]);
            
            % Create the gripper model
            self.model = SerialLink(link, 'name', self.plyFileNameStem);
        end
    end
end
