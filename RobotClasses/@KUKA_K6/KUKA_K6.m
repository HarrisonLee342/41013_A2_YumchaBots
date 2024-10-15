classdef KUKA_K6 < RobotBaseClass    
% Class for the Kuka K6 R900 robot created for Industrial Robotics Spring
% 2024 - Lab Assignment 2 

% Students contributed: 
% Sean Kim - 14282040
% Harrison Lee - 13935857
    
    properties(Access = public)   
        plyFileNameStem = 'KUKA_K6';
    end
    
    methods
        %% Constructor
        function self = KUKA_K6(baseTr,useTool,toolFilename)
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
            % link(1) = Link('d',0.33825,'a',0.12652,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            % link(2) = Link('d',0.019332,'a',0.34,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',pi/2);
            % link(3) = Link('d',-0.025,'a',-0.02,'alpha',pi/2,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(4) = Link('d',0.33,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(5) = Link('d',-0.013,'a',0,'alpha',pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(6) = Link('d',0.068,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0); 

            link(1) = Link([0      0.33825  0.12652 pi/2  0]);
            link(2) = Link([0      0.019332 0.34     0    0]);
            link(3) = Link([0     -0.025   -0.02    pi/2  0]);
            link(4) = Link([0      0.33     0      -pi/2  0]);
            link(5) = Link([0     -0.013    0       pi/2  0]);
            link(6) = Link([0      0.068    0        0    0]);

            % Joint limits
            link(1).qlim = [-90 90]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-90 90]*pi/180;  
            link(6).qlim = [-360 360]*pi/180;

            self.model = SerialLink(link,'name',self.name);
            
        end

    end
end
