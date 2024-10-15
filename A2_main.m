classdef A2_main < handle
    % Class created by Harrison Lee - 13935857

    properties(Constant)
        %% Constant variables

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
                        self.testingCartMovement();
                        % self.testingArmMovement();

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

                %% TESTING

                % dumplingPose = self.env.dumplingsInitial{4} * trotx(pi) * transl(0,0,-0.14);
                % self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);
                % 
                % disp('end effector')
                % disp(self.env.kuka.model.fkine(self.env.kuka.model.getpos).T)
                % 
                % for i = 1:size(self.env.dumplingsInitial,2)
                %     fprintf('Dumpling no: %d \n',i);
                %     disp(self.env.dumplingsInitial{i});
                % end
                % disp(size(self.env.dumplingsInitial));

                % Moving arm to plate, then moving plate to inside the cart
                for i = 1:size(self.env.platesInitial,2)
                    for j = 1:size(self.env.platesInitial,2)
                        platePose = self.env.platesInitial{j} * trotx(pi) * transl(0,0,0.02);
                        self.move.armMove(platePose, self.env.ur3.model, self.steps);
                    end

                    for k = 1:size(self.env.platesInitial,2)
                        platePose = self.env.carts.model{1}.base.T * transl(0,0,0.075) * trotx(pi);
                        self.move.objectMove(platePose, self.env.ur3.model, self.steps, self.env.plates, 1);
                    end
                    self.env.plates.model{1}.base = self.env.carts.model{1}.base.T * transl(0,0,0.075);
                    self.env.plates.model{1}.animate(0)
                end
              
            end

            function testingCartMovement(self)
                
                pose = transl(1, 0.5, 0);
                self.move.cartMove(self.env.cartUR3.model, pose, 75);
         
            end

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

            end
        end
    end


