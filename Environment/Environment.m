classdef Environment < handle
    % Class created by Harrison Lee - 13935857
    
    properties(Constant)
        %% Initialise constant variables

        % Location of robotic arms
        origin = transl(0,0,0);

        cartsInitial = [[0.8, 0, 0]; ...    % self.cart.objectModel{1} UR3 robotic arm
                       [-0.8, 0, 0]];       % self.cart.objectModel{2} KUKA robotic arm

        steps = 50;

    end

    properties
        %% Initialise changing variables
        % Robotic arm UR3
        ur3Origin
        ur3;
        ur3Gripper;
        ur3GripperOrigin;

        % Rbotic arm KUKA
        kukaOrigin;
        kuka;
        kukaGripper;
        kukaGripperOrigin;

        % % Origin point for robotic arms
        % ur3Origin;
        % kukaOrigin;

        % Holder for initial states for robotic arms
        initialStateUR3;
        initialStateKuka;

        % Objects
        plates;
        numPlates;
        plateStart;
        platePlace;
        platesInitial;
        platesFinal;

        dumplings;
        numDumplings;
        dumplingStart;
        dumplingPlace;
        dumplingsInitial;
        dumplingsFinal;

        cartUR3;
        cartUR3Poses;
        cartUR3Origin;
        cartKUKA;
        cartKUKAPoses;
        cartKUKAOrigin;
        numCarts;
        % CartsInitial;

        chairs;
        numChairs;
        chairsInitial;
        
        % Range Plot
        plot;

        % Testing
        testgripper

    end

    methods

        % % Testing class comment out after use
        % function self = Environment()
        %     self.loadEnvironment();
        %     pause(1);
        %     while true
        %         input_val = input('0 to exit: ', 's');
        %         switch input_val
        %             case '0'
        %                 break;
        %         end
        %     end
        % end

        function loadEnvironment(self)
            clf;

            surf([-1.25,1.25; -1.25,1.25] ...
                ,[-1.25,-1.25; 1.25,1.25] ...
                ,[0,0; 0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ... % Concrete from canvas
                ,'FaceColor','texturemap');
            axis equal;
            axis tight;
            view(3);
            hold on;

            % Round table (length = 1.5m, width = 1.2m, height = 0.3m) placed at [0, 0, 0.01]
            PlaceObject('rectangle_table.ply',[0,0,0.01]);
            
            % Chairs
            % PlaceObject('chair.ply', [0, 0.45, 0.01]);
            self.numChairs = 3;
            self.chairsInitial = self.calcSideBySidePose([-0.5,0.6,0.01], 0.5, self.numChairs, 'x');
            self.chairs = Objects('chair', self.numChairs);
            for i = 1:self.numChairs
                self.chairs.model{i}.base = self.chairsInitial{i};
                self.chairs.model{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % Plate (Radius = 0.079)
            % PlaceObject('plate.ply', [0, 0.4, 0.325]);
            self.plateStart = [0, 0.15, 0.31];
            self.platePlace = [0.5,0.45,0.31];
            self.numPlates = 3;
            self.platesFinal = self.calcSideBySidePose(self.platePlace, -0.5, self.numPlates, 'x');
            self.platesInitial = self.calcStackedPose(self.plateStart, 0.005, self.numPlates);
            self.plates = Objects('plate', self.numPlates);
            for i = 1:self.numPlates
                self.plates.model{i}.base = self.platesInitial{i};
                self.plates.model{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % Dumpling tray (Radius = 0.055m, height = 0.035m)
            % PlaceObject('dumpling_tray.ply', [0, 0.2, 0.33]);
            self.dumplingStart = [0, -0.15, 0.31];
            self.dumplingPlace = [-0.5,0.3,0.31];
            self.numDumplings = 3;
            self.dumplingsFinal = self.calcSideBySidePose(self.dumplingPlace, 0.5, self.numDumplings, 'x');
            self.dumplingsInitial = self.calcStackedPose(self.dumplingStart, 0.04, self.numDumplings);
            self.dumplings = Objects('dumpling', self.numDumplings);
            for i = 1:self.numDumplings
                self.dumplings.model{i}.base = self.dumplingsInitial{i};
                self.dumplings.model{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % Placing UR3 robotic arm ontop of the cart
            self.ur3Origin = transl(0.375,0,0.31);
            self.ur3 = UR3(self.ur3Origin);

            % Placing KUKA robotic arm ontop of the cart
            self.kukaOrigin = transl(-0.375,0,0.31);
            self.kuka = KUKA_K6(self.kukaOrigin);

            % Emergency stop button from UTS Toolbox
            PlaceObject('emergencyStopWallMounted.ply', [-0.75,0.6,0.2]);

            % Fire Extinguisher from UTS Toolbox
            PlaceObject('fireExtinguisher.ply', [-1,-1,0.01]);

            % % testing gripper
            % self.testgripper = LinearUR3eGripper(transl(0,0,0.5));



        end

        function positions = calcStackedPose(self, basePosition, stackHeight, num)
            % Calculate stacked positions for objects along the Z-axis (stacking them vertically)
            % Inputs:
            % - basePosition: Base [x, y, z] coordinates of the first object
            % - stackHeight: Height offset between each object in the stack
            % - num: Number of objects to be stacked

            positions = cell(1, num);

            for i = 1:num
                zPos = basePosition(3) + stackHeight * (num - i); % Increment Z-coordinate for stacking

                xPos = basePosition(1);
                yPos = basePosition(2);

                positions{i} = transl(xPos, yPos, zPos);
            end
        end

        function positions = calcSideBySidePose(self, basePosition, spacing, num, axis)
            % Calculate positions for objects placed side by side along a specified axis
            % Inputs:
            % - basePosition: Base [x, y, z] coordinates of the first object
            % - spacing: Distance between each object
            % - num: Number of objects to be placed
            % - axis: Axis for side by side placement ('x' or 'y')

            positions = cell(1, num);

            for i = 1:num
                if axis == 'x'
                    xPos = basePosition(1) + spacing * (i - 1); % Increment X-coordinate
                    yPos = basePosition(2);
                elseif axis == 'y'
                    xPos = basePosition(1);
                    yPos = basePosition(2) + spacing * (i - 1); % Increment Y-coordinate
                else
                    error('Axis must be either ''x'' or ''y''.');
                end

                zPos = basePosition(3); % Z-coordinate remains constant

                positions{i} = transl(xPos, yPos, zPos);
            end
        end
            
        function clearObjects(self)
            delete(self.chairs);
            delete(self.plates);
            delete(self.dumplings);
            
        end

        function RangePlot(self, arm)

            stepRads = deg2rad(45); 

            % stepValue = 0.05;               % Steps for the prismatic joint
            qlim = arm.qlim;

            pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
            % pointCloudeSize = floor((qlim(1,2)-qlim(1,1))/stepValue + 1) * ...
            %                           prod(floor((qlim(2:6,2)-qlim(2:6,1))/stepRads + 1));


            pointCloud = zeros(pointCloudeSize,3);

            counter = 1;
            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2) 
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                q6 = 0;
                                q = [q1,q2,q3,q4,q5,q6];
                                % fkineUTS 10 times faster
                                % tr = self.LUR3e.model.fkine(q).T;
                                tr = arm.fkineUTS(q); % Endeffector Forward Kinematics
                                pointCloud(counter,:) = tr(1:3,4)';
                                counter = counter + 1;
                                if mod(counter/pointCloudeSize * 100,1) == 0
                                    disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                end
                            end
                        end
                    end
                end
            end

            % Plotting 3D model showing where the end effector can be over all these samples
            self.plot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            axis tight;
            
            % % Take the MAX and MIN point from point cloud matrix
            % maxPoint = max(pointCloud); % [x, y, z]; x=1, y=2, z=3
            % minPoint = min(pointCloud); % [x, y, z]; x=1, y=2, z=3
            % 
            % radius = (maxPoint(1) + abs(minPoint(1)))/2;
            % volume = (radius^3)*4/3*pi;
            % 
            % disp(['Volume: ',num2str(volume), 'm^3  |  Radius: ', num2str(radius), 'm;']);
            % 
            % maxDistance = norm(maxPoint - self.origin);
            % 
            % disp(['Robotic arms maximum reach: ', num2str(maxDistance), 'm;']);
            % 
            % input('Workspace Calulation Completed');
        end
    end

end