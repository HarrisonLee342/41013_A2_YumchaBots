% classdef Collision
%     properties
%         ur3LinkCenters % Property to store the centers of the UR3 links
%         ur3Ellipsoids  % Property to store ellipsoids for each link
%         chairMesh      % Property to store chair mesh data
%     end
%
%     methods
%         function self = Collision()
%             % Constructor
%             self.ur3LinkCenters = {};
%             self.ur3Ellipsoids = {};
%             self.chairMesh = struct('vertices', [], 'faces', []);
%         end
%
%         function self = createChairMesh(self, plyFilePath)
%             % Load chair mesh from ply file
%             [vertexData, faceData, ~] = plyread(plyFilePath, 'tri');
%
%             % Check if the vertexData and faceData are valid
%             if isempty(vertexData) || isempty(faceData)
%                 error('Vertex data or face data is empty. Please check the PLY file.');
%             end
%
%             % Create a struct for chairMesh with vertices and faces
%             self.chairMesh.vertices = vertexData;
%             self.chairMesh.faces = faceData;
%         end
%
%         % Method to create ellipsoids for the UR3 links
%         % Add this to createLinkEllipsoids method to visualize each ellipsoid
%         function self = createLinkEllipsoids(self, ur3Model, linkRadii)
%             numLinks = size(ur3Model.base, 3);
%
%             figure(1); % Ensure you're working with figure 1
%             hold on;   % Keep the current plot
%
%             for j = 1:numLinks-1
%                 % Get the position of the previous and current joints
%                 p_prev = transl(ur3Model.fkine(ur3Model.getpos(j-1)));
%                 p_curr = transl(ur3Model.fkine(ur3Model.getpos(j)));
%
%                 % Calculate the midpoint of the link
%                 self.ur3LinkCenters{j} = (p_prev + p_curr) / 2;
%
%                 % Define the ellipsoid for the link
%                 center = self.ur3LinkCenters{j};
%                 radii = linkRadii{j};
%
%                 % Output debugging information
%                 disp(['Ellipsoid center: ', num2str(center)]);
%                 disp(['Ellipsoid radii: ', num2str(radii)]);
%
%                 % Create ellipsoid using parametric equations
%                 [u, v] = meshgrid(linspace(0, 2*pi, 50), linspace(0, pi, 50));
%                 X = radii(1) * cos(u) .* sin(v) + center(1);
%                 Y = radii(2) * sin(u) .* sin(v) + center(2);
%                 Z = radii(3) * cos(v) + center(3);
%
%                 % Plot the ellipsoid
%                 surf(X, Y, Z, 'FaceAlpha', 1, 'EdgeColor', 'none'); % Opaque ellipsoid
%             end
%
%             % Ensure the plot gets updated
%             axis equal;
%             drawnow;
%         end
%
%
%
%         % Method for collision detection
%         function isCollision = detectCollision(self, ur3Ellipsoids, chairMesh)
%             % Initialize collision status
%             isCollision = false;
%
%             % Access vertices and faces from chairMesh
%             chairVertices = self.chairMesh.vertices;
%             chairFaces = self.chairMesh.faces;
%
%             % Iterate through each UR3 ellipsoid
%             for i = 1:numel(self.ur3Ellipsoids)
%                 ur3Ellipsoid = self.ur3Ellipsoids{i};
%
%                 % Iterate through each face of the chair mesh
%                 for j = 1:size(chairFaces, 1)
%                     v1 = chairVertices(chairFaces(j, 1), :);
%                     v2 = chairVertices(chairFaces(j, 2), :);
%                     v3 = chairVertices(chairFaces(j, 3), :);
%
%                     % Check for intersection
%                     if self.ellipsoidTriangleIntersection(ur3Ellipsoid, v1, v2, v3)
%                         isCollision = true;
%                         return;
%                     end
%                 end
%             end
%         end
%
%         function isIntersect = ellipsoidTriangleIntersection(self, ellipsoid, v1, v2, v3)
%             % Compute the triangle normal
%             e1 = v2 - v1;
%             e2 = v3 - v1;
%             normal = cross(e1, e2);
%             d = -dot(normal, v1);
%
%             % Distance from the ellipsoid center to the plane of the triangle
%             dist = abs(dot(normal, ellipsoid.center) + d) / norm(normal);
%
%             % Check if the distance is less than the ellipsoid's radii
%             isIntersect = dist < max(ellipsoid.radii);
%         end
%     end
% end
%
%
classdef Collision
    properties
        planeNormal   % Normal vector of the tabletop plane
        planePoint    % A point on the tabletop plane
    end

    methods
        function self = Collision()
            % Constructor: Define the tabletop plane with a normal vector and point
            self.planeNormal = [0, 0, 1]; % Assuming the plane is flat on the table (z-plane)
            self.planePoint = [0, 0, 0.310]; % Adjust according to the height of your table
        end

        function intersectionPoints = checkLinePlaneIntersection(self, lineStartPoint, lineEndPoint)
            % Calculate the point of intersection between a line and the plane
            [intersectionPoints, check] = LinePlaneIntersection(self.planeNormal, self.planePoint, lineStartPoint, lineEndPoint);

            % Display the result
            if check == 1
                disp('Line intersects with the plane at:');
                disp(intersectionPoints);
            elseif check == 0
                disp('No intersection with the plane.');
            elseif check == 2
                disp('Line lies in the plane.');
            elseif check == 3
                disp('Line does not intersect within the segment.');
            end
        end

        function plotPlane(self)
            % Visualise the tabletop as a plane
            [X, Y] = meshgrid(-2:0.1:2, -2:0.1:2);
            Z = repmat(self.planePoint(3), size(X, 1), size(X, 2)); % Set Z as the table height
            surf(X, Y, Z, 'FaceAlpha', 0.5); % Draw the plane with transparency
            hold on;
            axis equal;
        end

        function plotLine(self, lineStartPoint, lineEndPoint, intersectionPoints)
            % Plot the line and the intersection point
            plot3(lineStartPoint(1), lineStartPoint(2), lineStartPoint(3), 'g*'); % Start point in green
            plot3(lineEndPoint(1), lineEndPoint(2), lineEndPoint(3), 'r*'); % End point in red
            plot3([lineStartPoint(1), lineEndPoint(1)], [lineStartPoint(2), lineEndPoint(2)], [lineStartPoint(3), lineEndPoint(3)], 'k'); % Line
            if ~isempty(intersectionPoints)
                plot3(intersectionPoints(1), intersectionPoints(2), intersectionPoints(3), 'k*', 'MarkerSize', 20); % Intersection point in black
            end
        end

        function collisionDetected = checkCollision(self, predictedJointPositions, robotModel, tableHeight, bufferDistance)
            % Check if the robot's predicted positions collide with the table, including a buffer
            collisionDetected = false;

            % Loop through each link in the robot to check for collision
            for linkIdx = 1:robotModel.n
                % Get the transformation matrix of each link relative to the base
                linkPoseSE3 = robotModel.A(1:linkIdx, predictedJointPositions);
                linkPoseMatrix = linkPoseSE3.T; % Convert SE3 to transformation matrix
                linkPosition = linkPoseMatrix(1:3, 4); % Extract the position of the link

                % Check if the Z-coordinate (height) of the link is below the table height + buffer
                if linkPosition(3) <= tableHeight + bufferDistance
                    disp(['Collision detected at link ', num2str(linkIdx), ' within buffer distance.']);
                    collisionDetected = true;
                    break; % Exit the loop as soon as a collision is detected
                end
            end
        end
    end
end
