classdef arcSegment_v4 < handle
    properties
        TurningCenter
        Radius
        StartPoint
        EndPoint
        allX
        allY
        tangents
        numPoints
    end
    
    methods
        function obj = arcSegment_v4(turningCenter, radius, startPoint, endPoint,curv_sign)
            %left is positive
            obj.TurningCenter = turningCenter;
            obj.Radius = radius;
            obj.StartPoint = startPoint;
            obj.EndPoint = endPoint;
            obj.numPoints = 200;
            obj.curv_sign = curv_sign;
            obj.generateArc();
        end
        
        function generateArc(obj)
            % Plot the arc segment
            
            % Calculate the angles corresponding to the start and end points
            angleStart = atan2(obj.StartPoint(2) - obj.TurningCenter(2), obj.StartPoint(1) - obj.TurningCenter(1));
            angleEnd = atan2(obj.EndPoint(2) - obj.TurningCenter(2), obj.EndPoint(1) - obj.TurningCenter(1));
            
            % Generate points along the arc
            t = linspace(angleStart, angleEnd, obj.numPoints)';
            if obj.curv_sign > 0 % turning left
                obj.tangents = t + pi/2;
            else % turning right
                obj.tangents = t - pi/2;
            end
            obj.allX = obj.TurningCenter(1) + obj.Radius * cos(t);
            obj.allY = obj.TurningCenter(2) + obj.Radius * sin(t);
            

        end
        function plotArc(obj)
                        % Plot the arc
            plot(obj.allX, obj.allY, 'b-', 'LineWidth', 2);
            
            % Plot the turning center
            hold on;
            plot(obj.TurningCenter(1), obj.TurningCenter(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
            
            % Plot the start and end points
            plot(obj.StartPoint(1), obj.StartPoint(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
            plot(obj.EndPoint(1), obj.EndPoint(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
            
            % Set axis equal for better visualization
            axis equal;
            
            % Add labels
            xlabel('X-axis');
            ylabel('Y-axis');
            title('Arc Segment');
            
            % Add legend
            legend('Arc', 'Turning Center', 'Start Point', 'End Point');
        end
    end
end

