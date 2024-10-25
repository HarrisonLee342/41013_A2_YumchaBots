classdef A2_main < handle
    % Class created by Harrison Lee - 13935857

    properties(Constant)
        %% Constant variables
        steps = 50;
    end

    properties
        %% Changing variables
        env;
        move;
        gripper;
        Col;  % Collision Detector
    end

    methods
        function self = A2_main()

            self.loadFiles();
            self.setClassVariables();

            % Load environment
            self.env.loadEnvironment();

            % Initialize the chair mesh for collision detection
            % self.env.chairMesh = self.Col.createChairMesh('plyFiles/chair.ply');

            % Display the chair mesh structure for debugging
            % disp(self.env.chairMesh);
            %
            % disp(fieldnames(self.env.chairMesh));  % Show the fields of chairMesh
            % disp(self.env.chairMesh);  % Show the whole structure

            while true
                input_val = input('test or 0 to exit: ', 's');
                switch input_val
                    case 'tc'
                        self.testingCartMovement();
                    case 't'
                        self.testingArmMovement();
                    case 'tcol'
                        self.testingCollision();
                    case '0'
                        disp('Exiting...');
                        break;
                end
            end
        end
    end

    methods
        function testingArmMovement(self)
            % Moving arm to plate, then moving plate to inside the cart
            for i = 1:size(self.env.platesInitial,2)
                for j = 1:size(self.env.platesInitial,2)
                    platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.02);

                    % Move the UR3 arm to the plate pose
                    self.move.armMove(platePose, self.env.ur3.model, self.steps)

                end
            end

            % Move plates to cart after checking for collisions
            for k = 1:size(self.env.platesInitial,2)
                platePose = self.env.carts.model{1}.base.T * transl(0,0,0.075) * trotx(pi);
                self.move.objectMove(platePose, self.env.ur3.model, self.steps, self.env.plates, 1);
            end
            self.env.plates.model{1}.base = self.env.carts.model{1}.base.T * transl(0,0,0.075);
            self.env.plates.model{1}.animate(0)
        end
    end
end

function testingCartMovement(self)
pose = transl(1, 0.5, 0);
self.move.cartMove(self.env.cartUR3.model, pose, 75);
end

% function testingCollision(self)
%     % Simulate arm movement and check for collisions
%     for j = 1:size(self.env.platesInitial,2)
%         platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.02);
%         self.move.armMove(platePose, self.env.ur3.model, self.steps);
%
%         % Create ellipsoids around UR3's links
%         self.Col.createLinkEllipsoids(self.env.ur3.model, {[0.1, 0.1, 0.1], [0.04, 0.04, 0.08], [0.03, 0.03, 0.06]});
%         disp('Creating Link Ellipsoids...');
%
%
%         % Check if there's a collision between UR3 and the chair
%         if self.Col.detectCollision(self.Col.ur3Ellipsoids, self.env.chairMesh)
%             disp('Collision detected with the chair!');
%             break;
%         end
%     end
% end



function testingCollision(self)
% Simulate arm movement and check for collisions
for j = 1:size(self.env.platesInitial,2)
    platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.2);

    % Move the UR3 arm towards the plate step by step, checking for collisions
    for step = 1:self.steps
        % Compute the current end-effector position during this step
        self.move.armMove(platePose, self.env.ur3.model,self.steps);

        % % Get the current position of the end-effector (last joint)
        % endEffectorPose = self.env.ur3.model.fkine(self.env.ur3.model.getpos());

        jointPositions = self.env.ur3.model.getpos();
        disp('Joint positions:');
        disp(jointPositions);
        % Get the forward kinematics (end-effector pose)
        endEffectorPoseSE3 = self.env.ur3.model.fkine(jointPositions);

        % Convert SE3 object to a 4x4 matrix
        endEffectorPose = endEffectorPoseSE3.T;

        % Perform the size check
        rows = size(endEffectorPose, 1);
        cols = size(endEffectorPose, 2);
        disp('End effector pose (should be 4x4 matrix):');
        disp(endEffectorPose);  % Print the entire matrix
        disp(['Rows: ', num2str(rows), ', Cols: ', num2str(cols)]);

        % Check if the matrix is indeed 4x4
        if rows == 4 && cols == 4
            endEffectorPosition = endEffectorPose(1:3, 4);  % Extract the position
            disp('End effector position (x, y, z):');
            disp(endEffectorPosition);
        else
            error(['Error: endEffectorPose is not a 4x4 matrix. It is ', num2str(rows), 'x', num2str(cols)]);
        end

        % Define the start and end points of the line (using end-effector position)
        lineStartPoint = endEffectorPosition';   % Start point at the current end-effector position
        lineEndPoint = [endEffectorPosition(1:2)', 0.0];  % End point (assume vertical line down to z=0)


        disp ('Linepoints:');
        disp (lineStartPoint);
        disp (lineEndPoint);


        % Check for intersection with the plane
        intersectionPoints = self.Col.checkLinePlaneIntersection(lineStartPoint, lineEndPoint);

        % If the end-effector crosses the plane (i.e., intersection exists and end-effector z < plane height)
        if ~isempty(intersectionPoints) && endEffectorPosition(3) <= 0.310  % Assuming plane is at z = 0.310
            disp('Collision detected with the plane during arm movement!');

            % Stop the movement by breaking the loop
            break;
        end
    end

    % If collision occurred, break out of the outer loop as well
    if exist('collisionDetected', 'var') && collisionDetected
        break;
    end
end

% Define the start and end points of the line for plotting
lineStartPoint = [0, 0, 0.310];  % Starting point of the line (above the table)
lineEndPoint = [0, 0, 0.0];      % End point of the line (below the table or on the surface)

% Plot the plane and the line with the intersection point
hold on;
self.Col.plotPlane();  % Plot the tabletop plane
self.Col.plotLine(lineStartPoint, lineEndPoint, intersectionPoints);  % Plot the line and intersection point
hold off;

disp('Line and plane intersection test complete.');
end


function setClassVariables(self)
self.env = Environment();
self.move = Movement();
self.Col = Collision();  % Initialize Collision class
end


methods(Static)

function loadFiles()
% Load necessary files and folders
if ~contains(path, 'RobotClasses')
    addpath('RobotClasses');
    disp('RobotClasses Folder loaded.');
end
if ~contains(path, 'Environment')
    addpath('Environment');
    disp('Environment Folder loaded.');
end
if ~contains(path, 'plyFiles')
    addpath('plyFiles');
    disp('Ply files folder loaded.');
end
if ~contains(path, 'Objects')
    addpath('Objects');
    disp('Object Folder loaded.');
end
if ~contains(path, 'Movement')
    addpath('Movement');
    disp('Movement Folder loaded.');
end
if ~contains(path, 'Collision')
    addpath('Collision');
    disp('Collision Folder loaded.');
end
end

function reloadFiles()
addpath('Environment');
addpath('plyFiles');
addpath('Objects');
addpath('RobotClasses');
addpath('Movement');
addpath('Collision');
end







