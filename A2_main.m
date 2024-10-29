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

        % Arduino Connection 
        arduinoObj;
        buttonPin = 'D2';

    end

    methods
        cla;
        function self = A2_main()

            self.loadFiles();
            self.setClassVariables();

            self.env.loadEnvironment();

            self.arduinoObj = arduino('COM5', 'Nano3');  
            configurePin(self.arduinoObj, self.buttonPin, 'pullup'); 

            while true

                input_val = input('test or 0 to exit: ', 's');

                switch input_val
                    case 't'
                        self.testingArmMovement();

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
                
                %% DUMPLINGS
                % for i= 1:size(self.env.dumplingsInitial,2)
                %     dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,-0.14);
                %     self.move.armMove(dumplingPose, self.env.kuka.model, self.steps);
                % 
                %     dumplingFinalPose = self.env.dumplingsFinal{i} * trotx(pi) * transl(0,0,-0.14);
                %     self.move.objectMove(dumplingFinalPose, self.env.kuka.model, self.steps, self.env.dumplings, i);
                % 
                % end
                
                %% PLATES
                % for i= 1:size(self.env.platesInitial,2)
                %     platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,-0.14);
                %     self.move.armMove(platePose, self.env.ur3.model, self.steps);
                %     if self.isButtonPressed()
                %         disp("Button pressed! Stopping movement.");
                %         return;  % Exit the function to halt further movement
                %     end
                %     plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,-0.14);
                %     self.move.objectMove(plateFinalPose, self.env.ur3.model, self.steps, self.env.plates, i);
                %     if self.isButtonPressed()
                %         disp("Button pressed! Stopping movement.");
                %         return;  % Exit the function to halt further movement
                %     end
                % end

                    %% Button Press Detection Loop
                    while true
                        if self.isButtonPressed()
                            disp('STOP');
                            break; % Exit the loop or take necessary action
                        end
                
                        for i= 1:size(self.env.platesInitial,2)
                            platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,-0.14);
                            self.move.armMove(platePose, self.env.ur3.model, self.steps);
        
                            plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,-0.14);
                            self.move.objectMove(plateFinalPose, self.env.ur3.model, self.steps, self.env.plates, i);
                            
                        end
                    end
                end
            
            
                function pressed = isButtonPressed(self)
                    % Read the button state
                    buttonState = readDigitalPin(self.arduinoObj, self.buttonPin);
                    pressed = (buttonState == 0);
                end
    
                function setupArduino(self)
                    % Clear existing Arduino object if it exists
                    try
                        if exist('self.arduinoObj', 'var') && ~isempty(self.arduinoObj)
                            clear self.arduinoObj;  % Clear the existing Arduino object
                            disp('Cleared existing Arduino connection.');
                        end
                    catch ME
                        disp('Error clearing Arduino object:');
                        disp(ME.message);
                    end
                
                    % Create Arduino object
                    try
                        self.arduinoObj = arduino('COM5', 'Nano3'); 
                        disp('Arduino connected.');
                
                        % Configure the button pin as pull-up
                        configurePin(self.arduinoObj, self.buttonPin, 'Pullup');
                    catch ME
                        disp('Error connecting to Arduino:');
                        disp(ME.message);
                    end
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


