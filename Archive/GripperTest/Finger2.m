classdef Finger2 < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'Finger2';  % The stem name for the PLY files
    end
    
    methods
        function self = Finger2()
            % Initialize the base model
            self.CreateModel();

            % Plot and color the robot using PLY files
            self.PlotAndColourRobot();
        end

        function CreateModel(self)
            % Define a "dummy" link for the base
               link(1) = Link('d', 0.20143, 'a', 0.03366, 'alpha', pi/2, 'qlim', [0,0]);
            link(2) = Link('d', 0, 'a', 0, 'alpha',0, 'qlim', [-pi/4, pi/4]);
            % Create the base model
            self.model = SerialLink(link, 'name', self.plyFileNameStem);
        end
    end
end
