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
        platesInitial;

        dumplings;
        numDumplings;
        dumplingsInitial;

        carts;
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

            % Round table (dia = 1.01167, r = 0.5 height = 0.3225) placed at [0, 0, 0.01]
            PlaceObject('table.ply',[0,0,0.01]);
            
            % Chairs
            % PlaceObject('chair.ply', [0, 0.45, 0.01]);
            self.numChairs = 8;
            self.chairsInitial = self.calcPose(0.45, 0.01, [0,0,0], self.numChairs);
            self.chairs = Objects('chair', self.numChairs);
            for i = 1:self.numChairs
                self.chairs.objectModel{i}.base = self.chairsInitial{i};
                self.chairs.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % Plate (Radius = 0.079)
            % PlaceObject('plate.ply', [0, 0.4, 0.325]);
            self.numPlates = 8;
            self.platesInitial = self.calcPose(0.4, 0, [0,0,0.325], self.numPlates);
            self.plates = Objects('plate', self.numPlates);
            for i = 1:self.numPlates
                self.plates.objectModel{i}.base = self.platesInitial{i};
                self.plates.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % Dumpling tray (Radius = 0.055m, height = 0.035m)
            % PlaceObject('dumpling_tray.ply', [0, 0.2, 0.33]);
            self.numDumplings = 6;
            self.dumplingsInitial = self.calcPose(0.2, 0.01, [0,0,0.32], self.numDumplings);
            self.dumplings = Objects('dumpling', self.numDumplings);
            for i = 1:self.numDumplings
                self.dumplings.objectModel{i}.base = self.dumplingsInitial{i};
                self.dumplings.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            % cart (lenght = 0.448, width = 0.225, height = 0.381)
            % height for arm is 0.344
            % height for plate/dishes is 0.075
            % PlaceObject('cart.ply', [0.65, 0, 0]);
            self.numCarts = 2;
            self.carts = Objects('cart', self.numCarts);
            for i = 1:self.numCarts
                self.carts.objectModel{i}.base = SE3(self.cartsInitial(i,:)).T;
                self.carts.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            self.ur3Origin = self.carts.objectModel{1}.base.T * transl(0,0,0.35);
            self.ur3 = UR3Modified(self.ur3Origin);

            self.kukaOrigin = self.carts.objectModel{2}.base.T * transl(0,0,0.35);
            self.kuka = KUKA_K6(self.kukaOrigin);

            % % testing gripper
            % self.testgripper = LinearUR3eGripper(transl(0,0,0.5));



        end

        function positions = calcPose(self, distance, height, origin, num)
            % Calculate evenly spaced positions around a circle (table)
            % Inputs:
            % - distance: Radius/distance from the origin
            % - height: Height at which the object will place offset from origin
            % - origin: origin point [x, y, z], also the point of rotation
            % - num: Number of objects to be created

            positions = cell(1, num);

            for i = 1:num
                % Calculate the angle for each plate (evenly spaced around a circle)
                theta = (i - 1) * (2 * pi/num); % Divides the circle into equal angles

                % Calculate the X and Y positions based on the angle and distance
                xPos = origin(1) + distance * cos(theta);  % X-coordinate
                yPos = origin(2) + distance * sin(theta);  % Y-coordinate
                zPos = origin(3) + height;                 % Z-coordinate, height is offset
                
                angleCorrection = theta - pi/2; % Rotate 180 degrees to face inward

                % Transformation matrix for the plate's position
                positions{i} = transl(xPos, yPos, zPos) * trotz(angleCorrection);
            end
        end

        function clearObjects(self)
            delete(self.chairs);
            delete(self.plates);
            delete(self.dumplings);
            delete(self.carts);
            
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