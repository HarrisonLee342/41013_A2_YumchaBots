classdef Movement < handle
    % Class created by Harrison Lee - 13935857

    properties(Constant)
        %% Constant variables

    end

    properties
        %% Changing variables

        % Final/initial position
        % pose;
        % Testing


    end

    methods
        function armMove(self, pose, arm, steps)
            % Moves the robotic arm to the desired position
            % Inputs:
            % - Pose: Selected position in transl format
            % - Arm: Robotic arm model (self.env.kuka.model or self.env.ur3.model)
            % - Steps: How smooth you want the animation to be, higher is smoother

            joints = arm.ikcon(pose, arm.getpos);

            q = jtraj(arm.getpos,joints, steps);
            for j = 1:size(q,1)
                arm.animate(q(j,:));
                drawnow();
                pause(0.01);
                
            end
        end

        function trapezoidalMovement(self, arm, startPose, endPose, t_total)
            
           steps = 100;
           t = linspace(0, t_total, steps); % Time vector for LSPB 

           qMatrix = nan(steps, length(startPose)); 

           % Generate trapezoidal trajectory from start to end pose over t
           for i = 1:length(startPose)

               qMatrix(:, i) = lspb(startPose(i), endPose(i), t);  % Interpolated joint configuration

           end

           % Animate the robotic arm movement
           for j = 1:steps
                % Animate the arm using the current joint configuration
                arm.animate(qMatrix(j, :));
                
                % Update the figure to reflect the changes in real-time
                drawnow();
                
                % Pause to control animation speed (syncs with the total time)
                pause(t_total / steps);
           end
        
        end


        % Sets variables with classes
        function setClassVariables(self)

        end
        function objectMove(self, pose, arm, steps, object, objectIndex)
            % Moves the robotic arm to the desired position
            % Inputs:
            % - Pose: Selected position in transl format
            % - Arm: Robotic arm model (self.env.kuka.model or self.env.ur3.model)
            % - Steps: How smooth you want the animation to be, higher is smoother
            % - Object: Object the robotic arm is holding


            joints = arm.ikcon(pose, arm.getpos);

            q = jtraj(arm.getpos,joints, steps);
            for j = 1:size(q,1)
                arm.animate(q(j,:));

                objectGripPose = arm.fkine(arm.getpos).T * transl(0, 0, 0.02);
                object.model{objectIndex}.base = objectGripPose;
                object.model{objectIndex}.animate(0);
                
                drawnow();
                pause(0.01);
                
            end
        end

        function cartMove(self, cart, pose, steps)
            % Moves the cart so that the object is within range of the robotic arm
            % Inputs:
            % - cart: Desired Cart
            % - pose: Desired Position to get to
            % - Steps: How smooth you want the animation to be, higher is smoother
            
            joints = cart.ikcon(pose, cart.getpos);

            q = jtraj(cart.getpos, joints, steps);

            for j = 1:size(q,1)
                cart.animate(q(j,:));
                drawnow();
                pause(0.01);
            end
        end
    end
end


