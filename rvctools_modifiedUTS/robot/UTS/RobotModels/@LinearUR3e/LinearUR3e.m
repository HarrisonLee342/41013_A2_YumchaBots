classdef LinearUR3e < RobotBaseClass
    %% LinearUR3e UR3e on a non-standard linear rail created by Harrison Lee - 13935857
    %
    % A custom model that was created by modifying the UR3e model 
    % to include a linear rail, that was copied over from LinearUR5 model
    % Place into RobotModels folder in rvctools
    
    properties(Access = public)   
        plyFileNameStem = 'LinearUR3e'; % Created by Harrison Lee 13935857
    end
    
    methods
        %% Constructor
        function self = LinearUR3e(baseTr,useTool,toolFilename)
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
			self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2); % Adjusted base for rail alignment like in linearUR5
            self.model.tool = self.toolTr;
			warning('The DH parameters are correct. But as of July 2023 the ply files for this LinearUR3e model are definitely incorrect, since we are using the UR3e ply files renamed as LinearUR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();

            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            % Adding a prismatic joint for the first link to represent the linear rail
            % link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            % link(2) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            % link(3) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            % link(4) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(5) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(7) = Link('d',0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            % Adding a PRISMATIC joint for the first link to represent the linear rail
            link(1) = Link([pi     0         0        pi/2  1]); % PRISMATIC Link
            link(2) = Link([0      0.15185   0        pi/2  0]);
            link(3) = Link([0      0        -0.24355   0    0]);
            link(4) = Link([0      0        -0.2132    0    0]);
            link(5) = Link([0      0.13105   0        pi/2  0]);
            link(6) = Link([0      0.08535   0       -pi/2	0]);
            link(7) = Link([0      0.0921    0         0    0]);

            % Incorporate joint limits
            link(1).qlim = [-0.8 -0.01]; % Limits for the PRISMATIC joint from Linear UR5
            % link(1).qlim = [-0.8 0]; % Limits for the PRISMATIC joint
            % link(2).qlim = [-360 360]*pi/180;
            link(2).qlim = [-90 90]*pi/180;   % Reduced joint limit to reduce 360 rotation for cleaner movements
            % link(3).qlim = [-360 360]*pi/180;
            link(3).qlim = [-90 90]*pi/180;   % Reduced joint limit to prevent arm driving into the ground
            % link(4).qlim = [-360 360]*pi/180;
            link(4).qlim = [-90 90]*pi/180;   % Reduced joint limit to prevent arm driving into the ground
            link(5).qlim = [-360 360]*pi/180;
            % link(6).qlim = [-360 360]*pi/180;
            link(6).qlim = [-90 90]*pi/180;   % Reduced joint limit to reduce 360 rotation for cleaner movements
            link(7).qlim = [-360 360]*pi/180;
            
            % Offset to make robotic arm stand up in desired initial position
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end