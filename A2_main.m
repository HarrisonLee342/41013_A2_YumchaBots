classdef A2_main < handle

    properties(Constant)
        %% Constant variables

    end

    properties
        %% Changing variables

        % Evironment class
        Env
        
        % Testing
        gripper

    end

    methods
        function self = A2_main()

            self.loadFiles();
            self.setClassVariables();

            self.Env.loadEnvironment();
            drawnow();
            
            self.test();


        end
    end

    methods
        function test(self)
            % Robotic arm does move
            for i = 1:size(self.Env.platesInitial,1)
                brickPose = transl(self.Env.platesInitial(i,:)) * trotx(pi) * transl(0,0,-0.14);

                brickJoints = self.Env.ur3.model.ikcon(brickPose,self.Env.ur3.model.getpos);

                qBrickInitial = jtraj(self.Env.ur3.model.getpos,brickJoints,50);
                for j = 1:size(qBrickInitial,1)
                    self.Env.ur3.model.animate(qBrickInitial(j,:));
                    drawnow();
                    pause(0.01);
                end
            end
            % Gripper does load in this way after the folder path is added
            self.gripper = LinearUR3eGripper(transl(0,0,0.33));
        end

        % Sets variables with classes
        function setClassVariables(self)

            %Environment Class
            self.Env = Environment();

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


