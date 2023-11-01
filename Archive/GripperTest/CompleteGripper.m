classdef CompleteGripper < handle
    properties
        leftFinger;
        rightFinger;
    end

    methods
        function self = CompleteGripper()
            % Constructor
            
            % Initialize the left and right fingers
            self.leftFinger = GripperFinger();
            self.rightFinger = Finger2();

            % Adjust the base of the rightFinger
            % Assuming a translational offset, adjust as needed
        %     self.rightFinger.model.base = transl(-0.03366*2, 0, 0);
         end

    %     function display(self)
    %         % Display the complete gripper
    % 
    %         % Plot the left and right fingers
    %         self.leftFinger.model.plot(zeros(1, self.leftFinger.model.n));
    %         self.rightFinger.model.plot(zeros(1, self.rightFinger.model.n));
    % 
    %         % Optionally, you can adjust the camera view, lighting, etc.
    %     end
    end
end