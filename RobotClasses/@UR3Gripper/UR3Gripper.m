classdef UR3Gripper < RobotBaseClass
    %% LinearUR3e robot's gripper created by Harrison Lee - 13935857
    % based off LinearUR3e robot gripper
    % Gripper sourced from https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button
    % A custom model that was created by downloading a 2F-85 gripper and splicing the model into base and fingers
    % Origin point moved to desired location per model and exported as a .ply file
    % Height of gripper is 0.09, to the base of the hand, 0.11 puts it between fingers
    % Base    = LinearUR3eGripperLink0.ply
    % Finger1 = LinearUR3eGripperLink1.ply
    % Finger2 = LinearUR3eGripperLink2.ply
    % Placed into RobotModels folder in rvctools
    % Axis have all been rearranged, Z axis rotates to close grippers, offset via Y axis fixed in blender

    % LOG GRIPPER ANIMATION:
    % Gripper Axis Attempt 1: Spliced model, axis varied but fixed using DH parameters, axis of rotation for the
    %                               gripper to close on was the y axis - NO SOLUTION FOUND
    % Gripper Axis Attempt 2: Changed the axis around, by switching the x and y axis, axis of rotation for the gripper
    %                               to close on was now the x axis had to add y offset into ply file - ROTATION CAN BE 
    %                               CHANGE VIA ALPHA VALUE (need to rotate model for fitting onto arm)
    % Gripper Axis Attempt 3: reordered the axis with z now being the axis of rotation, no offsets added into ply file
    %                               gripper can close via offset and trajectory - GRIPPERS CAN BE CLOSED AND OPENED
    %                               (need to rotate model for fitting onto arm)
    % Gripper Animate Attempt 1: tried to make a open and close function error with arrays when calling function
    %                               from the assignment class
    % Gripper Animate Attempt 2: tried to make an animate trajectory like in moving the arm to the bricks for the
    %                               grippers instead ran into error with ikon(inverse kinematics) in optimoptions(line 108)
    % Gripper Animate Attempt 3: Hard coded open and close functions into gripper class WORKS


    properties(Access = public)   
        plyFileNameStem = 'UR3Gripper'; % Created by Harrison Lee 13935857
    end

    properties(Constant)
        openPose = [0 0];  
        closePose = [-deg2rad(3.25) deg2rad(12.25)];

        steps = 50;
    end

    properties
        % Default offsets
        linkOffsets = [0, 0]; 
    end

    methods
        %% Constructor
        function self = LinearUR3eGripper(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;

            % self.model.base = self.model.base.T * baseTr * trotz(pi/2) * troty(-pi/2); % testing

            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

         %% CreateModel
        function CreateModel(self)
            % Testing
            % axis([-0.25 0.25 -0.25 0.25 -0.25 0.25])
            % r = LinearUR3eGripper
            % b = PlaceObject('HalfSizedRedGreenBrick.ply',[0,0,0.11])
            % verts = [get(b,'Vertices'), ones(size(get(b,'Vertices'),1),1)] * trotz(pi/2);
            % set(b,'Vertices',verts(:,1:3))
            % delete(b)
          
            % d = z-axis, a = x-axis

            % link(1) = Link('d',0.055,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0);
            % link(2) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0);
            link(1) = Link('d',0,'a',0.055,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',self.linkOffsets(1));
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',self.linkOffsets(2));
            self.model = SerialLink(link,'name',self.name);
        end 
        
        % Updating new link offsets
        function UpdateLinkOffsets(self, newOffsets)
            
            % setting offsets as the new position values
            self.linkOffsets = newOffsets;

            % Recreate the model with updated offsets
            self.CreateModel();
        end

        % Trajectory calculation and animation
        function Trajectory(self, startPose, endPose)

            qTraj = jtraj(startPose, endPose, self.steps);
            
            for i = 1:size(qTraj, 1)
                self.model.animate(qTraj(i, :)); 
                drawnow();
                pause(0.01);
            end
            
            % Updating the link offsets
            self.UpdateLinkOffsets(endPose);
        end

        % OpenGripper function
        function OpenGripper(self) 
            self.Trajectory(self.closePose, self.openPose);
        end

        % CloseGripper function 
        function CloseGripper(self)
            self.Trajectory(self.openPose, self.closePose);
        end
    end
end