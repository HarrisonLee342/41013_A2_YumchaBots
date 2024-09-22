classdef Objects < handle
    %   MULTIPLE OBJECTS created by Harrison Lee - 13935857
    %   A class that creates a group of objects (dish, dumplings, cart)
    %   Based off the RobotCow.m file

    %#ok<*TRYNC>

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end

    properties
        %> Number of bricks
        num = 0;

        %> A cell structure of \c brickCount brick models
        objectModel;

        %> workspace in meters
        workspace = [8,8];

        %> Dimensions of the workspace
        workspaceDimensions;
    end

    methods
        %% Constructors
        function self = Objects(num, objectType)
            if 0 < nargin
                self.num = num;
            end

            self.workspaceDimensions = [self.workspace/2, self.workspace/2, ...
                self.workspace/2, self.workspace/2, ...
                0, self.maxHeight];

            % Create the required number of objects
            for i = 1:self.num
                % self.objectModel{i} = self.GetObjectModel(['brick',num2str(i)]);

                % Use objectType to determine which model to load (dish, dumplings, cart)
                self.objectModel{i} = self.GetObjectModel(objectType);

                % Object location for base
                basePose = transl(0,0,0);

                self.objectModel{i}.base = basePose;
                
                % Plot 3D model
                plot3d(self.objectModel{i},0,'workspace',self.workspaceDimensions,'view',[-45,45],'delay',0,'noarrow','nowrist');
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
            for index = 1:self.num
                handles = findobj('Tag', self.objectModel{index}.name);
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
        %% GetObjectModel
        function model = GetObjectModel(objectType)
            switch lower(objectType)

                % Plate (Radius = 0.079)
                case 'plate'
                    name = 'plate';
                    [faceData,vertexData] = plyread('plate.ply','tri');
                    link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
                % Dumplings (Radius = 0.065m, height = 0.025m)
                case 'dumplings'
                    name = 'dumplings';
                    [faceData,vertexData] = plyread('dumplings.ply','tri');
                    link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
                    
                % cart (lenght = 0.448, width = 0.225, height = 0.381)
                % height for arm is 0.344
                % height for plate/dishes is 0.075
                case 'cart'
                    name = 'cart';
                    [faceData,vertexData] = plyread('cart.ply','tri');
                    link1 = Link('alpha',0,'a',0,'d',0,'offset',0);

                otherwise
                    error('Unknown object type: %s', objectType);
            end

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