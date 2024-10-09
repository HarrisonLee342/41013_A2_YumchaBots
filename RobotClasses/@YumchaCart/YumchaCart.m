classdef YumchaCart < RobotBaseClass
    % Class for the cart robot created for Industrial Robotics Spring
    % 2024 - Lab Assignment 2

    % cart (lenght = 0.448, width = 0.225, height = 0.381)
    % height for arm is 0.344
    % height for plate/dishes is 0.075

    % Students contributed:
    % Harrison Lee - 13935857

    properties(Access = public)
        plyFileNameStem = 'YumchaCart';
    end

    methods
        %% Constructor
        function self = YumchaCart(baseTr,useTool,toolFilename)
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
            % link = Link([offset     d       a       alpha    'prismatic']);

            link(1) = Link([0      0.05   0.15    pi/2     0]);
            link(2) = Link([ 0     0.15   0    -pi/2   1]); % PRIMATIC LINK

            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-10 10]; % Limits for the PRISMATIC joint

            % link(1) = Link([ 0     0.001   0    -pi/2   1]); % PRIMATIC LINK
            % 
            % link(1).qlim = [-10 10]; % Limits for the PRISMATIC joint
            
            self.model = SerialLink(link,'name',self.name);
            
        end
    end
end
