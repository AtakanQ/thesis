classdef arcSegment < handle
   properties
      init_pos
      init_tan
      radius
      length
      theta
      curv_sign
      center
      final_angle
      x_coor
      y_coor
      unused_x_coor
      unused_y_coor
      numPointsPerSegment
   end


   methods
       function obj = arcSegment(init_pos,init_tan, radius, length,curv_sign)
         if nargin > 0
            obj.init_pos = init_pos;
            obj.init_tan = init_tan;
            obj.radius = radius;
            obj.length = length;
            obj.numPointsPerSegment = 20;
            if radius ~= Inf
                obj.theta = length / radius;
            end
            obj.curv_sign = curv_sign;

         end
       end
       function [x , y] = getXY(obj)
           if       obj.curv_sign > 0
               angles = linspace(obj.init_tan - pi/2,obj.init_tan - pi/2 + obj.theta,obj.numPointsPerSegment);
               obj.final_angle = obj.init_tan + obj.theta;
               unused_angles = linspace(obj.init_tan - pi/2 + obj.theta,obj.init_tan + 3*pi/2,200);

               obj.center(1) = obj.init_pos(1) - obj.radius*sin(obj.init_tan);
               obj.center(2) = obj.init_pos(2) + obj.radius*cos(obj.init_tan);

           elseif   obj.curv_sign < 0
               obj.final_angle = obj.init_tan - obj.theta;
               
               angles = linspace(obj.init_tan + pi/2,obj.init_tan + pi/2 - obj.theta,obj.numPointsPerSegment);
               unused_angles = linspace(obj.init_tan + pi/2 - obj.theta,obj.init_tan - 3*pi/2,200);
               obj.center(1) = obj.init_pos(1) + obj.radius*sin(obj.init_tan);
               obj.center(2) = obj.init_pos(2) - obj.radius*cos(obj.init_tan);
           else %LINE SEGMENT
                obj.center(1) = -1;
                obj.center(2) = -1;
                destx = obj.init_pos(1) + obj.length * cos(obj.init_tan);
                desty = obj.init_pos(2) + obj.length * sin(obj.init_tan);
                xvals = linspace(obj.init_pos(1),destx);
                yvals = linspace(obj.init_pos(2),desty);
                obj.x_coor = xvals;
                obj.y_coor = yvals;
                obj.final_angle = obj.init_tan;
                x = xvals;
                y = yvals;
                return
           end

           add_to_x = obj.radius * cos(angles);
           add_to_y = obj.radius * sin(angles);

           add_to_unusedx = obj.radius * cos(unused_angles);
           add_to_unusedy = obj.radius * sin(unused_angles);

           obj.x_coor = add_to_x + obj.center(1);
           obj.y_coor = add_to_y + obj.center(2);
           obj.unused_x_coor = add_to_unusedx + obj.center(1);
           obj.unused_y_coor = add_to_unusedy + obj.center(2);
           
           x = obj.x_coor;
           y = obj.y_coor;
%            A = [x' y'];
%            sum(sqrt( sum( abs( diff( A ) ).^2, 2 ) )) % curvature length
       end
       function plotWholeCircle(obj)
           figure;
           plot(obj.center(1),obj.center(2), "Marker","*","Color",[0 1 0]);
           hold on
           plot(obj.x_coor,obj.y_coor,"Color",[0 0 1]);
           hold on
           plot(obj.unused_x_coor,obj.unused_y_coor,"Color",[1 0 0])
           title('Whole Circle')
           xlabel('x')
           ylabel('y')
           axis equal
           grid on
       end
       function plotSegment(obj)
           figure;
           plot(obj.x_coor,obj.y_coor,"Color",[0 0 1]);
           hold on
           title('Arc Segment')
           xlabel('x')
           ylabel('y')
           axis equal
           grid on
       end
   end
end