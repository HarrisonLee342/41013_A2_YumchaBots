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

        % function placeFood(self)
        %     % DUMPLINGS
        %     for i= 1:size(self.env.dumplingsInitial,2)
        %         dumplingPose = self.env.dumplingsInitial{i} * trotx(pi) * transl(0,0,0);
        %         dumplingFinalPose = self.env.dumplingsFinal{i} * trotx(pi) * transl(0,0,0);
        % 
        %         kukaPose = [-0.375;0;0.31];
        % 
        %         kukaDist = norm(kukaPose(1:3) - dumplingFinalPose(1:3, 4));
        % 
        %         if kukaDist < 0.7
        %             arm = self.env.kuka.model;
        %         else
        %             arm = self.env.ur3.model;
        %         end
        % 
        %         self.move.armMove(dumplingPose, arm, self.steps);
        % 
        % 
        %         self.move.objectMove(dumplingFinalPose, arm, self.steps, self.env.dumplings, i, [0,0,0]);
        % 
        %         self.env.dumplings.model{i}.base = self.env.dumplingsFinal{i};
        %         self.env.dumplings.model{i}.animate(0);
        %         pause(0.1);
        %     end
        % end
        % 
        % function placePlates(self)
        %     for i= 1:size(self.env.platesInitial,2)
        %         platePose = self.env.platesInitial{i} * trotx(pi) * transl(0,0,0);
        %         plateFinalPose = self.env.platesFinal{i} * trotx(pi) * transl(0,0,0);
        % 
        %         ur3Pose = [0.375;0;0.31];
        % 
        %         ur3Dist = norm(ur3Pose(1:3) - plateFinalPose(1:3, 4));
        % 
        %         if ur3Dist < 0.6
        %             arm = self.env.ur3.model;
        %         else
        %             arm = self.env.kuka.model;
        %         end
        % 
        %         self.move.armMove(platePose, arm, self.steps);
        % 
        %         self.move.objectMove(plateFinalPose, arm, self.steps, self.env.plates, i, [0,0,0]);
        % 
        %         self.env.plates.model{i}.base = self.env.platesFinal{i};
        %         self.env.plates.model{i}.animate(0);
        %         pause(0.1);
        %     end
        % end

        function armMoveCol(self, pose, arm, steps, tableHeight)
            % Moves the robotic arm to the desired position with collision checking
            % Inputs:
            % - Pose: Target position in transl format
            % - Arm: Robotic arm model (self.env.kuka.model or self.env.ur3.model)
            % - Steps: Number of steps for smoother animation
            % - tableHeight: Height of the table to check for collisions
            % - bufferDistance: Extra distance above the table to avoid collision

            joints = arm.ikcon(pose, arm.getpos);
            q = jtraj(arm.getpos, joints, steps);

            for j = 1:size(q,1)
                % Update the arm's configuration to the current step's joint positions
                arm.animate(q(j,:));
                drawnow();
                pause(0.01);

                % Collision check at the current position
                for linkIdx = 3:arm.n
                    linkPose = arm.A(1:linkIdx, q(j,:)).T; % Get transformation of each link
                    linkPosition = linkPose(1:3, 4); % Extract position of the link

                    if linkPosition(3) <= tableHeight
                        disp(['Collision detected with the table at link ', num2str(linkIdx), ' at step ', num2str(j)]);
                        return; % Stop movement if collision is detected
                    end
                end
            end
        end

        function collisionOccurred = armCol(self, pose, arm, steps, tableHeight)
            % Moves the robotic arm to the desired position with collision checking
            % Outputs:
            % - collisionOccurred: Returns true if a collision was detected, otherwise false

            joints = arm.ikcon(pose, arm.getpos);
            q = jtraj(arm.getpos, joints, steps);
            collisionOccurred = false;

            for j = 1:size(q, 1)
                % Collision check at the current position before animating
                for linkIdx = 2:arm.n
                    linkPose = arm.A(1:linkIdx, q(j, :)).T; % Get transformation of each link
                    linkPosition = linkPose(1:3, 4); % Extract position of the link

                    if linkPosition(3) <= tableHeight
                        disp(['Collision detected with the table at link ', num2str(linkIdx), ' at step ', num2str(j)]);
                        collisionOccurred = true;
                        return; % Stop movement and return collision status if detected
                    end
                end

                % Only animate if no collision was detected
                arm.animate(q(j, :));
                drawnow();
                pause(0.01);
            end
        end

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

        function objectMove(self, pose, arm, steps, object, objectIndex, offset)
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

                objectGripPose = arm.fkine(arm.getpos).T * trotx(pi) * transl(offset);
                object.model{objectIndex}.base = objectGripPose;
                object.model{objectIndex}.animate(0);

                drawnow();
                pause(0.01);

            end
        end


    end
end


