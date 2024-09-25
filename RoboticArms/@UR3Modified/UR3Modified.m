classdef UR3Modified < RobotBaseClass
    %% Based off UR3 Universal Robot 3kg payload robot model - Modified by Harrison Lee 13935857
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    % Inclusion of cart as a prismatic link?

    properties(Access = public)   
        plyFileNameStem = 'UR3Modified';
    end
    
    methods
%% Constructor
        function self = UR3Modified(baseTr,useTool,toolFilename)
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
            % link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            % link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            % link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            link(1) = Link([0      0.1519   0         pi/2  0]);
            link(2) = Link([0      0        -0.24365   0    0]);
            link(3) = Link([0      0        -0.21325   0    0]);
            link(4) = Link([0      0.11235   0        pi/2  0]);
            link(5) = Link([0      0.08535   0       -pi/2	0]);
            link(6) = Link([0      0.0819    0         0    0]);
            
            link(1).offset = -pi;
            link(2).offset = -pi/2;
            link(4).offset = -pi/2;

            % % Adding a PRISMATIC joint for the first link to represent the linear rail
            % % link(1) = Link([pi     0         0        pi/2  1]); % PRISMATIC Link
            % link(2) = Link([0      0.15185   0        pi/2  0]);
            % link(3) = Link([0      0        -0.24355   0    0]);
            % link(4) = Link([0      0        -0.2132    0    0]);
            % link(5) = Link([0      0.13105   0        pi/2  0]);
            % link(6) = Link([0      0.08535   0       -pi/2	0]);
            % link(7) = Link([0      0.0921    0         0    0]);

            % % Incorporate joint limits
            % link(1).qlim = [-0.8 -0.01]; % Limits for the PRISMATIC joint from Linear UR5
            % % link(1).qlim = [-0.8 0]; % Limits for the PRISMATIC joint
            % % link(2).qlim = [-360 360]*pi/180;
            % link(2).qlim = [-90 90]*pi/180;   % Reduced joint limit to reduce 360 rotation for cleaner movements
            % % link(3).qlim = [-360 360]*pi/180;
            % link(3).qlim = [-90 90]*pi/180;   % Reduced joint limit to prevent arm driving into the ground
            % % link(4).qlim = [-360 360]*pi/180;
            % link(4).qlim = [-90 90]*pi/180;   % Reduced joint limit to prevent arm driving into the ground
            % link(5).qlim = [-360 360]*pi/180;
            % % link(6).qlim = [-360 360]*pi/180;
            % link(6).qlim = [-90 90]*pi/180;   % Reduced joint limit to reduce 360 rotation for cleaner movements
            % link(7).qlim = [-360 360]*pi/180;
            
            % % Offset to make robotic arm stand up in desired initial position
            % link(3).offset = -pi/2;
            % link(5).offset = -pi/2;
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
