classdef A2_main < handle

    properties(Constant)
        %% Constant variables
        % fork test carlos
    end

    properties
        %% Changing variables

        % Evironment class
        Env

        % UR3 Robotic Arm
        ur3

        % Kuka Robotic Arm
        kuka

    end

    methods
        function self = A2_main()

            self.loadFiles();
            self.setClassVariables();

            self.Env.loadEnvironment();
            
            
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
        end

        % Reloading folder paths for debugging
        function reloadFiles()
            
            % Evironment
            addpath('Environment');
            
            % Ply files folder
            addpath('plyFiles');

            % Object class
            addpath('Objects');

        end
    end

    methods
        
        % Sets variables with classes
        function setClassVariables(self)

            %Environment Class
            self.Env = Environment();

        end

        
        
    end
end


