classdef arcSegment_v3
    properties
        Center % Center coordinates [x, y]
        StartAngle % Start angle in degrees
        EndAngle % End angle in degrees
        Radius % Radius of the arc
        Coordinates % Computed (x, y) coordinates of the arc
        num_points
    end
    
    methods
        % Constructor
        function obj = arcSegment_v3(center, startAngle, endAngle, radius,num_points)
            obj.Center = center;
            obj.StartAngle = startAngle;
            obj.EndAngle = endAngle;
            obj.Radius = radius;
            obj.num_points = num_points;
            obj = obj.computeCoordinates();
        end
        
        % Method to compute arc coordinates
        function obj = computeCoordinates(obj)
            % Convert angles to radians

            if( (obj.StartAngle - obj.EndAngle) > 180)
                obj.EndAngle = obj.EndAngle + 360;
            end

            startAngleRad = deg2rad(obj.StartAngle);
            endAngleRad = deg2rad(obj.EndAngle);

            % Compute (x, y) coordinates of the arc
            theta = linspace(startAngleRad, endAngleRad, obj.num_points); % 100 points for the arc
            x = obj.Center(1) + obj.Radius * cos(theta);
            y = obj.Center(2) + obj.Radius * sin(theta);
            
            obj.Coordinates = [x; y]';
        end
        
        % Method to plot the arc
        function plotArc(obj)
            plot(obj.Coordinates(:, 1), obj.Coordinates(:, 2), 'b', 'LineWidth', 1);
            % hold on;
            % plot(obj.Center(1), obj.Center(2), 'ro'); % Plot center point
            % axis equal;
            % xlabel('X');
            % ylabel('Y');
            % title('Arc Segment');
            % grid on;
            % hold off;
        end
    end
end