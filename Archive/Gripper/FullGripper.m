classdef FullGripper < handle
    properties(Access = public)
        baseFinger1;
        finger2;
    end
    
    methods
        %% Constructor
        function self = FullGripper()
            % Initialize the base finger and the second finger
            self.baseFinger1 = BaseFinger1();
            self.finger2 = Finger2();
            
            % Plot the full gripper
            self.PlotFullGripper();
        end
        
        %% Plot Full Gripper
        function PlotFullGripper(self)
            % Call the plot methods for both fingers
            % This assumes that the PlotAndColourRobot method plots the robot in the current figure
                figHandle = figure();  % create a new figure and store its handle

           hold on
            self.baseFinger1.PlotAndColourRobot();
           
            self.finger2.PlotAndColourRobot();
        
            
        end
        
        % ... Add methods for controlling both fingers simultaneously or individually
        
    end
end
