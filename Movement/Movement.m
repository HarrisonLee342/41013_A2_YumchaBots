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

        function cartMove(self, cart, cartIndex, object, objectIndex, steps)
            % Moves the cart so that the object is within range of the robotic arm
            % Inputs:
            % - cart: Cart/vechicle robotic arm is on
            % - cartIndex: Which cart/vechicle number
            % - object: The object the robot arm wants to pickup/move
            % - objectIndex: The specific object number that the robot arm wants to pickup/move
            % - Steps: How smooth you want the animation to be, higher is smoother

            cartPose = cart.model{cartIndex}.base.T;

            objectPose = object.model{objectIndex}.base.T;

            % the range the cart needs to stop within robot arms reach
            offsetDistance = 0.1;

            % Calculate the target position for the cart z value should not change
            targetCartPosition = transl(objectPose(1,4), objectPose(2,4), cartPose(3,4));

            % rotation of the cart should be automatic, similar to a revolute link
            targetCartPose = targetCartPosition * trotz(pi/2);
            
            q = jtraj(cartPose, targetCartPose, steps);

            for j = 1:size(q,1)
                cart.model{cartIndex}.animate(q(j,:));
                drawnow();
                pause(0.01);
            end
        end
    end
end


