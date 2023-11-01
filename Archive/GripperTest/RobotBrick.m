classdef RobotBrick < handle
    properties (Constant)
        maxHeight = 10;  % Maximum height for plotting
    end
    
    properties
        brickCount = 1;  % Number of bricks
        brickModel;  % A cell structure of brick models
        paddockSize = [10,10];  % Size of the paddock in meters
        workspaceDimensions;  % Dimensions of the workspace
    end
    
    methods
      %% Constructor
function self = RobotBrick(brickCount)
    if 0 < nargin
        self.brickCount = brickCount;
    end

    self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                               ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                               ,0,self.maxHeight];

    % Create the required number of bricks
    for i = 1:self.brickCount
        self.brickModel{i} = self.GetBrickModel(['brick',num2str(i)]);
        % Set the brick to the identity matrix (this means it won't move from its original position in the ply file)
        self.brickModel{i}.base = SE3();  % This sets it to the identity matrix

        % Plot 3D model
        plot3d(self.brickModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
        if i == 1 
            hold on;
        end
    end

    axis equal
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end 
end
        %% Destructor
        function delete(self)
            for index = 1:self.brickCount
                handles = findobj('Tag', self.brickModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
        
        %% Random movement logic for the bricks
        function PlotSingleRandomStep(self)
            % [Your movement logic for the bricks]
        end    
        
        %% Test movement for the bricks
        function TestPlotManyStep(self,numSteps,delay)
            % [Your testing logic for the bricks]
        end
    end
    
    methods (Static)
        %% GetBrickModel
        function model = GetBrickModel(name)
            if nargin < 1
                name = 'Brick';
            end
            [faceData,vertexData] = plyread('brick.ply','tri');  % Assuming you have a brick.ply file
            link1 = Link('alpha',0,'a',0,'d',0.1,'offset',0);  % Adjust parameters as needed
            model = SerialLink(link1,'name',name);
            
            model.faces = {[], faceData};
            model.points = {[], vertexData};
        end
    end    
end
