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

                objectGripPose = arm.fkine(arm.getpos).T * trotx(pi) * transl(0, 0, -0.14);
                object.model{objectIndex}.base = objectGripPose;
                object.model{objectIndex}.animate(0);
                
                drawnow();
                pause(0.01);
                
            end
        end

        function cartMove(self, cart, pose, object, objectnum, offset, steps)
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

                for i = 1:objectnum
                    height = 0.075 + (offset * i);
                    object{i}.base = cart.fkine(cart.getpos).T * transl(0, 0, height);
                    object{i}.animate(0);
                    drawnow();
                    pause(0.01);
                end
            end
            
            % function cartMove(self, cart, pose, steps)
            % % Moves the cart so that the object is within range of the robotic arm
            % % Inputs:
            % % - cart: Desired Cart
            % % - pose: Desired Position to get to
            % % - Steps: How smooth you want the animation to be, higher is smoother
            % 
            % joints = cart.ikcon(pose, cart.getpos);
            % 
            % q = jtraj(cart.getpos, joints, steps);
            % 
            % for j = 1:size(q,1)
            %     cart.animate(q(j,:));
            %     drawnow();
            %     pause(0.01);
            % 
            %     for i = 1:self.numPlates
            %             self.plates.model{i}.base = self.platesInitial{i};
            %             self.plates.model{i}.animate(0);
            %             drawnow();
            %             pause(0.01);
            %         end
            % end
        end
    end
end


