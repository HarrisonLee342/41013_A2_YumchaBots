classdef Assignment_1 < handle

    properties(Constant)
        %% Initialise constant variables
        
        % Location of robotic arm
        transOrigin = transl(0.5,0,0.02); % translation transformation matrix

        % Gripper orientation correction
        gripperOrienCor = trotz(pi/2) * troty(-pi/2);

        % Bricks are (L x W x H) = (0.133426 x 0.066713 x 0.033356)
        brickInitial  = [[-0.5, -0.40, 0.02]; ...
                        [-0.2, -0.40, 0.02]; ...
                        [-0.1, -0.40, 0.02]; ...
                        [0.0, -0.40, 0.02]; ...
                        [0.1, -0.40, 0.02]; ...
                        [0.2, -0.40, 0.02]; ...
                        [0.3, -0.40, 0.02]; ...
                        [0.4, -0.40, 0.02]; ...
                        [0.5, -0.40, 0.02]];

        % Bricks final position
        brickPlacement = [[-0.55, -0.15, 0.02]; ...
                        [-0.55, 0.00, 0.02]; ...
                        [-0.55, 0.15, 0.02]; ...
                        [-0.55, -0.15, 0.05]; ...
                        [-0.55, 0.00, 0.05]; ...
                        [-0.55, 0.15, 0.05]; ...
                        [-0.55, -0.15, 0.08]; ...
                        [-0.55, 0.00, 0.08]; ...
                        [-0.55, 0.15, 0.08]];

        % Number of steps for trajectory
        % steps = 125;
        % steps = 100;
        % steps = 75;
        steps = 50;
        % steps = 25;


    end

    properties
        %% Initialise changing variables
        % Robotic arm
        LUR3e;

        % Gripper
        LUR3eGripper;

        % Gripper point of attachment
        gripperOrigin;
        
        % Origin point for robotic arm
        origin; % convert transOrigin to a 1x3 matrix that holds the point

        % Holder for initial robotic arm state
        initialState;

        % Holds bricks for visuals
        vis_bricks;

        % Holds bricks for simulation
        bricks;

        % Debugging
        debug = false;

        % Refurbish Environment
        refurbish = false;
        
        % Workspace Calculation Plot holder, for clearing
        workspaceCalcPlot;

        % Plot clear automated
        plotted = false;

    end

    methods
        function self = Assignment_1()

            % Load Environment automatically
            self.originCoversion();
            self.Environment();

            while true
                
                % If brick placement has already run, reset bricks
                if self.refurbish
                    self.refurbishEnvironment();
                    disp('Refurbished');
                    self.refurbish = false;
                end

                if self.plotted
                    self.plotClear();
                    disp('Plot cleaned');
                    self.plotted = false;
                end

                input_val = input('Enter function or 0 to exit: ', 's');
                
                switch input_val
                    case 'b'
                        self.BrickPlacement();
                        disp('Brick Placement procedure initiating...');

                    case 'd'
                        self.debug = true;
                        disp('Debugging mode');

                    case 'c'
                        while true
                            plotCloudInput = input('Include gripper in calculations? (y/n): ', 's');
            
                            switch plotCloudInput
                                case 'y'
                                    disp('Workspace Calcuation with gripper initiated...');
                                    self.WorkspaceCalculation(true);
                                    break;

                                case 'n'
                                    disp('Workspace Calcuation without gripper initiated...');
                                    self.WorkspaceCalculation(false);
                                    break;

                                otherwise
                                    disp('Invalid input.');
                            end
                        end

                    case '0'
                        disp('Exiting...');
                        break;

                    otherwise
                        disp('Invalid input.');
                end
            end
        end
    end

    methods

        function originCoversion(self)
            self.origin = self.transOrigin(1:3, 4)';  % Extract and transpose the translation vector
        end

        function refurbishEnvironment(self)
            
            % Trajectory calculation for current arm positon to its initial position
            qReturn = jtraj(self.LUR3e.model.getpos, self.initialState, self.steps);
            for i = 1:size(qReturn,1)
                % Returns robotic arm to its initial state
                self.LUR3e.model.animate(qReturn(i,:));
                % Animate gripper moving with arm
                self.gripperOrigin = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * self.gripperOrienCor;
                self.LUR3eGripper.model.base = self.gripperOrigin;
                self.LUR3eGripper.model.animate(self.gripperOrigin);
                drawnow();
                pause(0.01);
            end

            % Deletes placed bricks
            delete(self.bricks);

            % Places visual bricks
            for i = 1:9
                self.vis_bricks(i) = PlaceObject('HalfSizedRedGreenBrick.ply' ...
                                        , self.brickInitial(i,:));
            end
        end

        function Environment(self)
            % Clear figure and command window
            clf;
            clc;
            
            surf([-1,3; -1,3] ...
                ,[-1,-1; 1,1] ...
                ,[0,0; 0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ... % Concrete from canvas
                ,'FaceColor','texturemap');
            axis equal;
            axis tight;
            view(3);
            hold on;

            % Place the robot and gripper
            self.LUR3e = LinearUR3e(self.transOrigin);

            self.gripperOrigin = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * self.gripperOrienCor;
            self.LUR3eGripper = LinearUR3eGripper(self.gripperOrigin);

            % Inital position stored for return
            self.initialState = self.LUR3e.model.getpos;
            
            % Place bricks for the environment
            for i = 1:9
                self.vis_bricks(i) = PlaceObject('HalfSizedRedGreenBrick.ply' ...
                                        , self.brickInitial(i,:));
            end

            % 'barrier1.5x0.2x1m.ply' from UTS Toolbox - 1.5(lenght) x 0.2(width) x 1(height)
            % Places fence at (-1, 0) at a length of 2m along the y-axis
            h_1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,-1,0]);
            verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,2) = verts(:,2) * (4/3);  % Adjusting length of fence to 2m
            set(h_1,'Vertices',verts(:,1:3))

            % Places fence at (1, 0) at a length of 2m along the y-axis
            h_2 = PlaceObject('barrier1.5x0.2x1m.ply',[0,1,0]);
            verts = [get(h_2,'Vertices'), ones(size(get(h_2,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,2) = verts(:,2) * (4/3);  % Adjusting length of fence to 2m
            set(h_2,'Vertices',verts(:,1:3))

            % Places 2 fences at (0, -1), (2, -1) at a length of 2m along the x-axis
            h_3a = PlaceObject('barrier1.5x0.2x1m.ply',[0,-1,0]);
            verts = [get(h_3a,'Vertices'), ones(size(get(h_3a,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * (4/3);  % Adjusting length of fence to 2m
            set(h_3a,'Vertices',verts(:,1:3))
            h_3b = PlaceObject('barrier1.5x0.2x1m.ply',[1.5,-1,0]);
            verts = [get(h_3b,'Vertices'), ones(size(get(h_3b,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * (4/3);  % Adjusting length of fence to 2m
            set(h_3b,'Vertices',verts(:,1:3))

            % Places 2 fences at (0, 1), (2, 1) at a length of 2m along the x-axis
            h_4a = PlaceObject('barrier1.5x0.2x1m.ply',[0,1,0]);
            verts = [get(h_4a,'Vertices'), ones(size(get(h_4a,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * (4/3);  % Adjusting length of fence to 2m
            set(h_4a,'Vertices',verts(:,1:3))
            h_4b = PlaceObject('barrier1.5x0.2x1m.ply',[1.5,1,0]);
            verts = [get(h_4b,'Vertices'), ones(size(get(h_4b,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * (4/3);  % Adjusting length of fence to 2m
            set(h_4b,'Vertices',verts(:,1:3))

            % Emergency stop button from UTS Toolbox
            PlaceObject('emergencyStopWallMounted.ply', [2.5,-0.9,0.75]);

            % Fire Extinguisher from UTS Toolbox
            PlaceObject('fireExtinguisher.ply', [2.8,0.8,0.02]);

            % Construction worker from UTS Toolbox, rotate by 180, location: (2.75,-0.5,0.02)
            constructionWorker = PlaceObject('personMaleConstruction.ply', [-2.75,0.5,0.02]);
            verts = get(constructionWorker, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            rotated_verts = verts_homogeneous * trotz(pi);
            set(constructionWorker, 'Vertices', rotated_verts(:, 1:3));
        end

        function BrickPlacement(self)
            
            % Place bricks that the robotic arm will move
            self.bricks = RobotBricks(9);
            for i = 1:size(self.brickInitial, 1)
                self.bricks.brickModel{i}.base = SE3(self.brickInitial(i,:)).T;
                self.bricks.brickModel{i}.animate(0);
            end
            
            % Delete bricks from the environment
            delete(self.vis_bricks);

             % Total number of bricks to process
            totalBricks = size(self.brickInitial, 1);
        
            % Initialize completion rate logger
            completedBricks = 0;
            completionRate = (completedBricks / totalBricks) * 100;

            % Robotic arm / brick movement logic
            for i = 1:size(self.brickInitial,1)

                % Process Status Update
                disp(['Processing Brick ', num2str(i), ' of ', num2str(totalBricks), ...
                    '  |  Completion Rate: ', num2str(completionRate), '%']);

                % Locaton of brick, grab brick from top, offset for the height of the brick (0.03) and gripper (0.11)
                brickPose = transl(self.brickInitial(i,:)) * trotx(pi) * transl(0,0,-0.14); 

                % Joints from current pose to brick
                brickJoints = self.LUR3e.model.ikcon(brickPose,self.LUR3e.model.getpos);
                
                % Calculate trajectory (current postion to brick pick up location)
                qBrickInitial = jtraj(self.LUR3e.model.getpos,brickJoints,self.steps);
                for j = 1:size(qBrickInitial,1)   
                    % Animate arm moving to brick
                    self.LUR3e.model.animate(qBrickInitial(j,:));

                    % Animate Gripper moving with arm
                    self.gripperOrigin = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * self.gripperOrienCor;
                    self.LUR3eGripper.model.base = self.gripperOrigin;
                    self.LUR3eGripper.model.animate(self.gripperOrigin);

                    drawnow();
                    pause(0.01);
                end
                

                % % Gripper close
                % gripBrickPose = transl(self.brickInitial(i,:)) * self.gripperOrienCor; 
                % gripBrickJoints = self.LUR3eGripper.model.ikcon(gripBrickPose,self.LUR3eGripper.model.getpos);
                % % Calculate trajectory (current postion to brick pick up location)
                % gripTraj = jtraj(self.LUR3eGripper.model.getpos,gripBrickJoints,self.steps);
                % for j = 1:size(gripTraj,1)   
                %     self.LUR3eGripper.model.animate(gripTraj(j,:));
                %     drawnow();
                %     pause(0.01);
                % end

                self.LUR3eGripper.CloseGripper;

                % Checking if values are right
                if self.debug
                    startBrickPose = self.brickInitial(i,:);
                    endEffectorPose = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T;
                    disp('Brick Current Pose:');
                    disp(startBrickPose);
                    disp('End Effector Pose:');
                    disp(endEffectorPose);
                end

                % Locaton of where brick will be placed, offset for the height of the brick (0.03) and gripper (0.11)
                brickPose2 = transl(self.brickPlacement(i,:)) * trotx(pi) * transl(0,0,-0.14);

                % Joints from current pose to brick location
                brickJoints2 = self.LUR3e.model.ikcon(brickPose2,self.LUR3e.model.getpos);

                % Calculate trajectory (current postion to brick drop off location)
                qBrickPlacement = jtraj(self.LUR3e.model.getpos,brickJoints2,self.steps);
                for k = 1:size(qBrickPlacement,1)
                    % Animate arm moving to brick placement location
                    self.LUR3e.model.animate(qBrickPlacement(k,:))
                    % Animate gripper moving with arm, offset by gripper
                    self.gripperOrigin = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * self.gripperOrienCor;
                    self.LUR3eGripper.model.base = self.gripperOrigin;
                    self.LUR3eGripper.model.animate(self.gripperOrigin);

                    % Animate bricks moving with the gripper (0.11)
                    brickPoseGripper = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * transl(0, 0, 0.11);
                    self.bricks.brickModel{i}.base = brickPoseGripper;
                    self.bricks.brickModel{i}.animate(0)
                    drawnow();
                    pause(0.01);
                end

                self.LUR3eGripper.OpenGripper;
                
                % Place bricks in the end location
                self.bricks.brickModel{i}.base = SE3(self.brickPlacement(i,:)).T;
                self.bricks.brickModel{i}.animate(0)

                % Update completion rate
                completedBricks = completedBricks + 1;
                completionRate = (completedBricks / totalBricks) * 100;
                disp(['Completed Brick  ', num2str(i), ' of ', num2str(totalBricks), ...
                            '  |  Completion Rate: ', num2str(completionRate), '%']);

                % Checking if values are right
                if self.debug
                    endBrickPose = self.brickPlacement(i,:);
                    endEffectorPose = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T;
                    disp('Brick Placement Pose:');
                    disp(endBrickPose);
                    disp('End Effector Pose:');
                    disp(endEffectorPose);
                end
                
                % Move arm back to previous position
                qBrickPlacement = flip(qBrickPlacement,1);
                for m = 1:size(qBrickPlacement,1)
                    % Animate arm going back to its previous position
                    self.LUR3e.model.animate(qBrickPlacement(m,:));
                    % Animate gripper moving with arm
                    self.gripperOrigin = self.LUR3e.model.fkine(self.LUR3e.model.getpos).T * self.gripperOrienCor;
                    self.LUR3eGripper.model.base = self.gripperOrigin;
                    self.LUR3eGripper.model.animate(self.gripperOrigin);
                    drawnow();
                end
            end

            % Resets the location of bricks
            self.refurbish = true;

            % No longer in debug state
            self.debug = false;

            input('Brick Placement Completed');
        end
        
        function WorkspaceCalculation(self, gripper)

            % If gripper, argument is not provided no gripper
            if nargin < 2
                gripper = false;
            end
            % From Tutorial 3 solution (adjusted for 7 joints)

            % fkineUTS: V9 Solution adapted to work with the V10 SE3 class (14/02/23)
            % Intended to be used in situations where Forward Kinematics needs to be
            % calculated many times in quick succession (e.g. calculating a point
            % cloud). Approximately 10x quicker than V10 SE3 fkine when convertToSE3 = false;. 

            %% fkine: takes 1600+ seconds / fkineUTS: takes 170 seconds - 0.98576
            % stepRads = deg2rad(30);
            
            %% fkine: takes 244.2025 seconds - 1.0955 / fkineUTS: takes 24 seconds - 1.0955
            stepRads = deg2rad(45); 

            %% fkine: takes 31 seconds / fkineUTS: takes 3 seconds - 0.89907
            % stepRads = deg2rad(60);

            stepValue = 0.05;               % Steps for the prismatic joint
            qlim = self.LUR3e.model.qlim;

            % Don't need to worry about joint 7, account for prismatic joint
            % pointCloudeSize = floor((qlim(1:6,2)-qlim(1:6,1))/stepRad + 1))
            pointCloudeSize = floor((qlim(1,2)-qlim(1,1))/stepValue + 1) * ...
                                      prod(floor((qlim(2:6,2)-qlim(2:6,1))/stepRads + 1));
            
            % % By iterating thorugh each step finds the total number of iterations (manual)
            % pointCloudeSize = 0;
            % for q1 = qlim(1,1):stepValue:qlim(1,2) % Prismatic joint step
            %     for q2 = qlim(2,1):stepRads:qlim(2,2)
            %         for q3 = qlim(3,1):stepRads:qlim(3,2)
            %             for q4 = qlim(4,1):stepRads:qlim(4,2)
            %                 for q5 = qlim(5,1):stepRads:qlim(5,2)
            %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
            %                         pointCloudeSize = pointCloudeSize + 1;
            %                     end
            %                 end
            %             end
            %         end
            %     end
            % end

            pointCloud = zeros(pointCloudeSize,3);

            counter = 1;
            tic
            
            for q1 = qlim(1,1):stepValue:qlim(1,2) % Adjusted for prismatic joint
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    % Don't need to worry about joint 7, just assume it=0
                                    q7 = 0;
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    % fkineUTS 10 times faster
                                    % tr = self.LUR3e.model.fkine(q).T;
                                    tr = self.LUR3e.model.fkineUTS(q); % Endeffector Forward Kinematics
                                    position = tr(1:3,4)';

                                    % If gripper is included, adjust the position
                                    if gripper
                                        gripperOffset = [0, 0, 0.11]; % gripper offset
                                        position = position + gripperOffset;
                                    end

                                    pointCloud(counter,:) = position;
                                    counter = counter + 1;
                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);                                        
                                    end
                                end
                            end
                        end
                    end
                end
            end

            % Plotting 3D model showing where the end effector can be over all these samples
            self.workspaceCalcPlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            axis tight
            
            % Take the MAX and MIN point from point cloud matrix
            maxPoint = max(pointCloud); % [x, y, z]; x=1, y=2, z=3
            minPoint = min(pointCloud); % [x, y, z]; x=1, y=2, z=3

            radius = (maxPoint(1) + abs(minPoint(1)))/2;
            volume = (radius^3)*4/3*pi;

            disp(['Volume: ',num2str(volume), 'm^3  |  Radius: ', num2str(radius), 'm;']);

            maxDistance = norm(maxPoint - self.origin);

            disp(['Robotic arms maximum reach: ', num2str(maxDistance), 'm;']);
            
            % Plot completed
            self.plotted = true;

            input('Workspace Calulation Completed');
        end

        function plotClear(self)
            delete(self.workspaceCalcPlot);
        end
    end
end


