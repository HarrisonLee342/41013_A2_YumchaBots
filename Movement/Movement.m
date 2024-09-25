classdef Movement < handle

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
        function self = Movement()
            
        end
    end

    methods
        function objectMovement(self, objectPose, arm, steps)

            pose = objectPose;
            joints = arm.ikcon(pose, arm.getpos);

            q = jtraj(arm.getpos,joints, steps);
            for j = 1:size(q,1)
                arm.animate(q(j,:));
                drawnow();
                pause(0.01);
            end
        end

    end
end


