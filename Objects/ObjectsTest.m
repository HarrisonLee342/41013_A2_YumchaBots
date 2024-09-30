classdef Objects < handle
    %   MULTIPLE OBJECTS created by Harrison Lee - 13935857
    %   A class that creates a group of objects (dish, dumplings, cart)
    %   Based off the files RobotCow.m and RobotBaseClass.m

    %#ok<*TRYNC>

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end

    properties
        %> Number of objects
        objNum;

        %> A cell structure of \c objNum object models
        model;

        %> workspace in meters
        workspace = [3,3];

        %> Dimensions of the workspace
        workspaceDimensions;
        
        % %> Ply file name
        % plyFileNameStem;
    end

    methods
        %% Constructors
        function self = Objects(objectType, num)
            if 0 < nargin
                self.objNum = num;
            end

            self.workspaceDimensions = [self.workspace/2, self.workspace/2, ...
                self.workspace/2, self.workspace/2, ...
                0, self.maxHeight];

            % Create the required number of objects
            for i = 1:num
                % Use objectType to determine which model to load (dish, dumplings, cart)
                self.model{i} = self.GetObjectModel(objectType, i);
                
                % % Ploting and colouring object
                % self.plotandcolour();

                % Object location for base
                basePose = transl(0,0,0);

                self.model{i}.base = basePose;

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
            for index = 1:self.objNum
                handles = findobj('Tag', self.model{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
    end

    methods(Static)
        %% GetObjectModel
        function model = GetObjectModel(objectType, num)

            name = [objectType, num2str(num)];

            switch lower(objectType)

                % Plate (Radius = 0.079)
                case 'plate'
                    link = Link('alpha',0,'a',0,'d',0,'offset',0);
                    
                % Dumplings (Radius = 0.055m, height = 0.035m)
                case 'dumpling'
                    link = Link('alpha',0,'a',0,'d',0,'offset',0);

                case 'chair'
                    link = Link('alpha',0,'a',0,'d',0,'offset',0);

                % cart (lenght = 0.448, width = 0.225, height = 0.381)
                % height for arm is 0.344
                % height for plate/dishes is 0.075
                case 'cart'
                    link(1) = Link([ 0     0   0   0   0]);
                    link(2) = Link([pi/2   0   0   0   1]); % PRIMATIC LINK
                    % link = Link('alpha',0,'a',0,'d',0,'offset',0);

                otherwise
                    error('Unknown object type: %s', [objectType, num2str(num)]);
            end

            model = SerialLink(link,'name', name);

            for linkIndex = 0:model.n
                [faceData,vertexData,plyData{linkIndex+1}] = plyread([objectType,'Link',num2str(linkIndex),'.ply'],'tri');
                model.faces{linkIndex+1} = faceData;
                model.points{linkIndex+1} = vertexData;
                % model.faces{linkIndex+1} = {faceData, []};
                % model.points{linkIndex+1} = {vertexData, []};
            end

            % model.faces = {faceData, []};
            % model.points = {vertexData, []};

            roughMinMax = sum(abs(model.d) + abs(model.a));
            workspacePlot = [-roughMinMax roughMinMax -roughMinMax roughMinMax -0.01 roughMinMax];

            model.plot3d(zeros(1,model.n),'noarrow','workspace',workspacePlot,'view',[-30,30]);

            model.delay =0;
            for linkIndex = 0:model.n
                handles = findobj('Tag', model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData.vertex.red ...
                        , plyData.vertex.green ...
                        , plyData.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end