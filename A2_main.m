classdef A2_main < handle
    % Class created by Harrison Lee - 13935857

    properties(Constant)
        %% Constant variables

        % Animation steps:
        steps = 100;
    end

    properties
        %% Changing variables

        % Evironment class
        env;

        % Movement class
        move;

        % Testing
        gripper;

        %Collision Detection
        Col;

    end

    methods
        function self = A2_main()

            self.loadFiles();
            self.setClassVariables();

            self.env.loadEnvironment();


            while true

                input_val = input('test or 0 to exit: ', 's');

                switch input_val
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

            % % UR3 to plate Movement
            % for i = 1:size(self.env.platesInitial,2)
            %     platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,-0.14);
            %     self.move.armMove(platePose, self.env.ur3.model, self.steps);
            % end

            % % KUKA to dumpling tray Movement
            for i= 1:size(self.env.dumplingsInitial,2)
                dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,-0.14);
                self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);
            end

            % % UR3 Range plot
            % self.env.RangePlot(self.env.ur3.model);

            % % KUKA Range plot
            % self.env.RangePlot(self.env.kuka.model);

            %% TESTING

            %% DUMPLINGS
            for i= 1:size(self.env.dumplingsInitial,2)
                dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,-0.14);
                self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);

                dumplingFinalPose = self.env.dumplingsFinal{i} * trotx(pi) * transl(0,0,-0.14);
                self.move.objectMove(dumplingFinalPose, self.env.kuka.model, self.steps, self.env.dumplings, i);

            end

            %% PLATES
            for i= 1:size(self.env.platesInitial,2)
                platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,-0.14);
                self.move.armMove(platePose, self.env.ur3.model, self.steps);

                plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,-0.14);
                self.move.objectMove(plateFinalPose, self.env.ur3.model, self.steps, self.env.plates, i);

            end
        end

        % function testingCollision(self)
        %     tableHeight = 0.310; % Define the table height for collision checks
        %     bufferDistance = 0.005; % Smaller buffer distance to allow closer moves
        %     lastSafePosition = self.env.ur3.model.getpos(); % Initial safe position
        %     collisionOccurred = false; % Flag to stop all animation after collision
        % 
        %     for j = 1:size(self.env.platesInitial, 2)
        %         if collisionOccurred
        %             break; % Stop further animation if a collision already occurred
        %         end
        % 
        %         platePose = self.env.platesInitial{j} * trotx(pi) * transl(0, 0, 0.-0.02);
        % 
        %         % Loop through each step of the arm movement
        %         for step = 1:self.steps
        %             % Predict the next position without moving
        %             predictedNextJointPositions = self.move.predictArmMove(platePose, self.env.ur3.model, step, self.steps);
        % 
        %             % Flag to track if a collision was detected in the prediction
        %             collisionAtStep = false;
        % 
        %             % Check each link for collision using predicted joint positions
        %             for linkIdx = 2:self.env.ur3.model.n
        %                 % Get transformation of each link at the predicted joint positions
        %                 linkPoseSE3 = self.env.ur3.model.A(1:linkIdx, predictedNextJointPositions);
        %                 linkPoseMatrix = linkPoseSE3.T; % Convert SE3 to transformation matrix
        %                 linkPosition = linkPoseMatrix(1:3, 4); % Extract position of the link
        % 
        %                 % Check if the Z-coordinate (height) of the link is below the table height
        %                 if linkPosition(3) <= tableHeight + bufferDistance
        %                     disp(['Collision detected with the table at link ', num2str(linkIdx), ' during step ', num2str(step)]);
        %                     collisionAtStep = true;
        %                     collisionOccurred = true; % Set global collision flag
        %                     break; % Exit the link loop on collision
        %                 end
        %             end
        % 
        %             if collisionAtStep
        %                 % If a collision was predicted, revert to last safe position
        %                 disp('Reverting to last safe position');
        %                 self.env.ur3.model.animate(lastSafePosition); % Only animate safe position
        %                 pause(0.1); % Small pause for reversion visualization
        %                 break; % Exit the step loop if collision was detected
        %             else
        %                 % Update the last safe position and proceed with animation if no collision
        %                 lastSafePosition = predictedNextJointPositions;
        %                 disp(['No collision detected for step ', num2str(step), ', moving to next position']);
        %                 pause(0.1)
        %                 self.env.ur3.model.animate(predictedNextJointPositions);
        %                 pause(0.1); % Small pause for animation step
        %             end
        %         end
        % 
        %         % Exit the outer loop if a collision was detected for the current plate
        %         if collisionOccurred
        %             disp(['Collision detected for plate ', num2str(j), ', stopping trajectory for this plate']);
        %             break;
        %         end
        %     end
        % 
        %     % Define the start and end points of the line for visualization
        %     lineStartPoint = [0, 0, tableHeight];
        %     lineEndPoint = [0, 0, 0.0];
        % 
        %     % Plot the plane and the line with the intersection point
        %     hold on;
        %     self.Col.plotPlane();
        %     self.Col.plotLine(lineStartPoint, lineEndPoint, []);
        %     hold off;
        % 
        %     disp('Predicted collision check complete.');

        function testingCollision(self)
            tableHeight = 0; % Define the table height for collision checks
            % bufferDistance = 0.005; % Smaller buffer distance to allow closer moves
            lastSafePosition = self.env.ur3.model.getpos(); % Initial safe position
            collisionOccurred = false; % Flag to stop all animation after collision

            for j = 1:size(self.env.platesInitial,2)
                platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.02);

                % Move the UR3 arm to the plate pose
                
                self.move.armMoveWithCollisionCheck(platePose, self.env.ur3.model, self.steps, tableHeight);
            end
        end




        % Sets variables with classes
        function setClassVariables(self)

            % Environment Class
            self.env = Environment();

            % Movement Class
            self.move = Movement();

            %Collision Class
            self.Col = Collision();
        end
    end

    methods(Static)

        % Loading necessary file paths / toolbox
        function loadFiles()

            % Robot classes
            if ~contains(path, 'RobotClasses')
                addpath('RobotClasses');
                disp('RobotClasses Folder loaded.');
            else
                disp('RobotClasses Folder already loaded.');
            end

            % Environment Class
            if ~contains(path, 'Environment')
                addpath('Environment');
                disp('Environment Folder loaded.');
            else
                disp('Environment Folder already loaded.');
            end

            % Ply files folder
            if ~contains(path, 'plyFiles')
                addpath('plyFiles');
                disp('Ply files folder loaded.');
            else
                disp('Ply files folder already loaded.');
            end

            % Object class
            if ~contains(path, 'Objects')
                addpath('Objects');
                disp('Object Folder loaded.');
            else
                disp('Object Folder already loaded.');
            end

            % Movement class
            if ~contains(path, 'Movement')
                addpath('Movement');
                disp('Movement Folder loaded.');
            else
                disp('Movement Folder already loaded.');
            end

            % Collision class
            if ~contains(path, 'Collision')
                addpath('Collision');
                disp('Collision Folder loaded.');
            else
                disp('Collision Folder already loaded.');
            end


        end

        % Reloading folder paths for debugging
        function reloadFiles()

            % Evironment
            addpath('Environment');

            % Ply files folder
            addpath('plyFiles');

            % Object class
            addpath('Objects');

            % Robot Classes
            addpath('RobotClasses');

            % Movement classck
            addpath('Movement');

            % Collision class
            addpath('Collision');

        end
    end
end