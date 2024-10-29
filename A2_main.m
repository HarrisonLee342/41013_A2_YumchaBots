classdef A2_main < handle
    % Class created by Harrison Lee - 13935857

    properties(Constant)
        %% Constant variables
        tableHeight = -0.01;
        % Animation steps:
        steps = 50;
    end

    properties
        %% Changing variables

        % Evironment class
        env;

        % Movement class
        move;

        % Testing
        gripper;

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
                        fprintf('runing');
                        self.testingArmMovement();

                    case 'tc'
                        self.testingCollision();

                    case 'd'
                        self.guiDumplings();

                    case 'p'
                        self.guiPlates();

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
                % for i= 1:size(self.env.dumplingsInitial,2)
                %     dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,-0.14);
                %     self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);
                % end

                % % UR3 Range plot
                % self.env.RangePlot(self.env.ur3.model);

                % % KUKA Range plot
                % self.env.RangePlot(self.env.kuka.model);

                % DUMPLINGS
                for i= 1:size(self.env.dumplingsInitial,2)
                    dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,0);
                    dumplingFinalPose = self.env.dumplingsFinal{i} * trotx(pi) * transl(0,0,0);

                    % kukaPose = [-0.375;0;0.31];
                    % 
                    % kukaDist = norm(kukaPose(1:3) - dumplingFinalPose(1:3, 4));
                    % 
                    % if kukaDist < 0.7
                    %     arm = self.env.kuka.model;
                    % else
                    %     arm = self.env.ur3.model;
                    % end

                    % self.move.armMove(dumplingPose, arm, self.steps);
                    % 
                    % self.move.objectMove(dumplingFinalPose, arm, self.steps, self.env.dumplings, i, [0,0,0]);

                    self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);

                    self.move.objectMove(dumplingFinalPose, self.env.kuka.model, self.steps, self.env.dumplings, i, [0,0,0]);

                    self.env.dumplings.model{i}.base = self.env.dumplingsFinal{i};
                    self.env.dumplings.model{i}.animate(0);
                    pause(0.1);
                end

                % PLATES
                for i= 1:size(self.env.platesInitial,2)
                    platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,0);
                    plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,0);
                    
                    ur3Pose = [0.375;0;0.31];

                    ur3Dist = norm(ur3Pose(1:3) - plateFinalPose(1:3, 4));
                    
                    if ur3Dist < 0.6
                        arm = self.env.ur3.model;
                    else
                        arm = self.env.kuka.model;
                    end

                    self.move.armMove(platePose, arm, self.steps);

                    self.move.objectMove(plateFinalPose, arm, self.steps, self.env.plates, i, [0,0,0]);

                    self.env.plates.model{i}.base = self.env.platesFinal{i};
                    self.env.plates.model{i}.animate(0);
                    pause(0.1);
                end

            end

            function guiDumplings(self)
                % DUMPLINGS
                for i= 1:size(self.env.dumplingsInitial,2)
                    dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,0);
                    dumplingFinalPose = self.env.dumplingsFinal{i} * trotx(pi) * transl(0,0,0);

                    kukaPose = [-0.375;0;0.31];

                    kukaDist = norm(kukaPose(1:3) - dumplingFinalPose(1:3, 4));

                    if kukaDist < 0.7
                        arm = self.env.kuka.model;
                    else
                        arm = self.env.ur3.model;
                    end

                    % self.move.armMove(dumplingPose, arm, self.steps);

                    self.move.armMoveCol(dumplingPose, arm, self.steps, self.tableHeight);


                    self.move.objectMove(dumplingFinalPose, arm, self.steps, self.env.dumplings, i, [0,0,0]);

                    self.env.dumplings.model{i}.base = self.env.dumplingsFinal{i};
                    self.env.dumplings.model{i}.animate(0);
                    pause(0.1);
                end
            end

            function guiPlates(self)
                % PLATES
                for i= 1:size(self.env.platesInitial,2)
                    platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,0);
                    plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,0);
                    
                    ur3Pose = [0.375;0;0.31];

                    ur3Dist = norm(ur3Pose(1:3) - plateFinalPose(1:3, 4));
                    
                    if ur3Dist < 0.6
                        arm = self.env.ur3.model;
                    else
                        arm = self.env.kuka.model;
                    end

                    % self.move.armMove(platePose, arm, self.steps);

                    self.move.armMoveCol(platePose, arm, self.steps, self.tableHeight);


                    self.move.objectMove(plateFinalPose, arm, self.steps, self.env.plates, i, [0,0,0]);

                    self.env.plates.model{i}.base = self.env.platesFinal{i};
                    self.env.plates.model{i}.animate(0);
                    pause(0.1);
                end
              
            end

            % function testingCollision(self)
            %     % % bufferDistance = 0.005; % Smaller buffer distance to allow closer moves
            %     % lastSafePosition = self.env.ur3.model.getpos(); % Initial safe position
            %     % collisionOccurred = false; % Flag to stop all animation after collision
            % 
            %     for j = 1:size(self.env.platesInitial,2)
            %         platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.02);
            % 
            %         self.move.armMoveWithCollisionCheck(platePose, self.env.ur3.model, self.steps, self.tableHeight);
            %     end
            % end

            % Sets variables with classes
            function setClassVariables(self)

                % Environment Class
                self.env = Environment();
   
                % Movement Class
                self.move = Movement();

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
                if ~contains(path, 'Collison')
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

                % Movement class
                addpath('Movement');

                % Collision class
                addpath('Collision');

            end
        end
    end


