classdef A2_main < handle

    properties(Constant)
        %% Constant variables

    end

    properties
        %% Changing variables

        % Evironment class
        Env

    end

    methods
        function self = A2_main()

            self.loadFiles();

            % self.Env.loadEnvironment();
            
            
        end
    end

    methods
        
        % Loading necessary file paths / toolbox
        function loadFiles(self)

            % If toolbox is not added then load the toolbox
            if exist('transl', 'file') ~= 2
                run('rvctools_modified/startup_rvc.m');
                disp('Robotics and Vision Toolbox(Modified) loaded.');
            else
                disp('Robotics and Vision Toolbox(Modified) already loaded.');
            end

            if exist('loadEnvironment','file') ~= 2
                addpath('Environment');
                disp('Environment Folder loaded.');
            else 
                disp('Environment Folder already loaded.');
            end
            self.Env = Environment();

            % if exist('','file') ~= 2
            %     addpath('Object');
            %     disp('Object Folder loaded.');
            % else 
            %     disp('Object Folder already loaded.');
            % end


        end
        
    end
end


