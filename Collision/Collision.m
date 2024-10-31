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
    end
end