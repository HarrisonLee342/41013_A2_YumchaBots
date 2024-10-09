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

            % link(1) = Link('alpha',0,'a',0,'d',0.001,'offset',0);
            % link(2) = Link([ 0     0.001   0   -pi/2   1]); % PRIMATIC LINK
            % 
            % link(2).qlim = [-10 10]; % Limits for the PRISMATIC joint

            link(1) = Link([ 0     0.001   0   -pi/2   1]); % PRIMATIC LINK

            link(1).qlim = [-10 10]; % Limits for the PRISMATIC joint from Linear UR5

            self.model = SerialLink(link,'name',self.name);

        end

        function moveToTarget(self, targetX, targetY)
            % Current robot base position
            currentPose = self.model.fkine(self.model.getpos); % Forward kinematics to get current pose

            % Extract current X, Y position from base transformation matrix
            currentX = currentPose.t(1);  % Current X position
            currentY = currentPose.t(2);  % Current Y position

            % Calculate the target angle to rotate towards the target
            targetAngle = atan2(targetY - currentY, targetX - currentX);

            % Set joint angles for rotation (link 1 is rotational)
            q1 = targetAngle;  % Set rotation joint angle

            % Calculate the distance to move linearly
            distance = sqrt((targetX - currentX)^2 + (targetY - currentY)^2);

            % Set joint value for prismatic joint (link 2)
            q2 = distance;  % Move the prismatic joint to the distance

            % Create joint trajectory for smooth movement
            steps = 50;  % Number of steps for the movement
            currentQ = self.model.getpos;  % Get current joint configuration
            targetQ = [q1, q2];  % Target joint configuration (rotation, linear move)

            % Create a smooth trajectory from the current joint config to the target
            qTrajectory = jtraj(currentQ, targetQ, steps);

            % Animate the movement
            for i = 1:steps
                self.model.animate(qTrajectory(i, :));
                drawnow;
                pause(0.01);
            end
        end

    end
end
