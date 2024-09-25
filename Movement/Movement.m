classdef Movement < handle

    properties(Constant)
        %% Constant variables

    end

    properties
        %% Changing variables
        
        % Testing
        

    end

    methods
        function self = Movement()
            
        end
    end

    methods
        function objectMovement(self)
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
end


