classdef RobotDumplings < handle
    %   ROBOT BRICKS created by Harrison Lee - 13935857
    %   A class that creates a group of bricks up to 9
    %   Based off the RobotCow.m file    
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of bricks
        brickCount = 9;
        
        %> A cell structure of \c brickCount brick models
        brickModel;
        
        %> workspace in meters
        workspace = [4,2];
        
        %> Dimensions of the workspace
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = RobotBricks(brickCount)
            if 0 < nargin
                self.brickCount = brickCount;
            end
            
            self.workspaceDimensions = [-1, 3, ...
                                        -1, 1, ...
                                        0, self.maxHeight];

            % Create the required number of bricks
            for i = 1:self.brickCount
                self.brickModel{i} = self.GetBrickModel(['brick',num2str(i)]);
                
                % Bricks location for base
                basePose = transl(0,0,0);

                self.brickModel{i}.base = basePose;
                
                %Plot 3D model
                plot3d(self.brickModel{i},0,'workspace',self.workspaceDimensions,'view',[-45,45],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end
        
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
    end
    
    methods (Static)
        %% GetBrickModel
        function model = GetBrickModel(name)
            if nargin < 1
                name = 'brick';
            end
            [faceData,vertexData] = plyread('HalfSizedRedGreenBrick.ply','tri');
            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);
          
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end