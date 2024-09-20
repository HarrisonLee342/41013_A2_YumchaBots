classdef Environment < handle

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

        steps = 50;

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


    end
    
    methods
        function self = Environment()
            % self.loadEnviroment();
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

        function originCoversion(self)
            self.origin = self.transOrigin(1:3, 4)';  % Extract and transpose the translation vector
        end

        function loadEnvironment(self)
            clf
            
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
    end

end