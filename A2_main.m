classdef A2_main < handle

    properties(Constant)
        %% Constant variables

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
                        self.test();

                    case '0'
                        disp('Exiting...');
                        break;

                end

            end
        end
    end

        methods
            function test(self)
                for i = 1:size(self.env.platesInitial,1)
                    platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,-0.14);
                    self.move.objectMovement(platePose, self.env.ur3.model, 50);
                end

                for i= 1:size(self.env.dumplingsInitial,1)
                    dumplingPose = self.env.dumplingsInitial{4} * trotx(pi) * transl(0,0,-0.14);
                    self.move.objectMovement(dumplingPose, self.env.kuka.model, 50);
                end
               
                % for i = 1:size(self.env.platesInitial,1)
                %     platePose = self.env.platesInitial(i,:).T * trotx(pi) * transl(0,0,-0.14);
                %     self.move.objectMovement(platePose, self.env.kuka.model, 50);
                % end

                % for i = 1:size(self.env.platesInitial,1)
                %     platePose = transl(self.env.platesInitial(i,:)) * trotx(pi) * transl(0,0,-0.14);
                %     self.move.objectMovement(platePose, self.env.kuka.model, 50);
                % end

            end

            % Sets variables with classes
            function setClassVariables(self)

                % Environment Class
                self.env = Environment();

                self.move = Movement();

            end
        end

        methods(Static)

            % Loading necessary file paths / toolbox
            function loadFiles()

                % If toolbox is not added then load the toolbox
                if ~contains(path, 'rvctools_modified')
                    run('rvctools_modifiedUTS/startup_rvc.m');
                    disp('Robotics and Vision Toolbox(Modified) loaded.');
                else
                    disp('Robotics and Vision Toolbox(Modified) already loaded.');
                end

                % Robotic Arm and Grippers class
                if ~contains(path, 'RoboticArms')
                    addpath('RoboticArms');
                    disp('RoboticArms Folder loaded.');
                else
                    disp('RoboticArms Folder already loaded.');
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

                % Robotic Arm and Grippers class
                addpath('RoboticArms');

                % Movement class
                addpath('Movement');

            end
        end
    end


