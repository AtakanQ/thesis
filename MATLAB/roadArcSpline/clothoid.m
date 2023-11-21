classdef clothoid < handle
   properties
      arcSegments
      arcX
      arcY
      init_pos
      init_tan
      init_curv
      final_curv
      length
      order
      curv_increment
      curv_sign
      centers
      allX
      allY
      final_tangent
      numPointsPerSegment
   end

   methods
       function obj = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               length,order,arcSegClass)
         if nargin > 0
            obj.init_pos = init_pos;
            obj.init_tan = init_tan;
            obj.init_curv = init_curvature;
            obj.final_curv = final_curvature;
            obj.length = length;
            obj.order = order;
            obj.curv_increment = (final_curvature - init_curvature) / order;
            obj.curv_sign = sign(final_curvature - init_curvature);
            obj.arcSegments = arcSegClass;
            obj.numPointsPerSegment = 20;
            if(obj.curv_sign == 0)
                error('Curvature is zero')
            end
         end
       end

       function generateArcSegments(obj)
           if obj.order <= 3
               error('Order is too low.')
           end

           obj.centers = zeros(obj.order+1,2);
           obj.arcX = zeros(obj.order+1,obj.numPointsPerSegment);
           obj.arcY = zeros(obj.order+1,obj.numPointsPerSegment);
           obj.arcSegments(1) = arcSegment(obj.init_pos,obj.init_tan,...
               abs(inv(obj.init_curv)),obj.length/(2 * obj.order),sign(obj.init_curv));
           [obj.arcX(1,:), obj.arcY(1,:)] = obj.arcSegments(1).getXY();
           obj.centers(1,:) = obj.arcSegments(1).center;


           for i = 2:(obj.order)
                curvature = obj.init_curv + (i-1) * (obj.final_curv - obj.init_curv) / obj.order;
                radius = abs(1/curvature);
                position = [obj.arcSegments(i-1).x_coor(end) obj.arcSegments(i-1).y_coor(end)];
                arcLen = obj.length/obj.order;
                start_angle = obj.arcSegments(i-1).final_angle;

                obj.arcSegments(i) = arcSegment(position,start_angle,radius,arcLen,sign(curvature));
                [obj.arcX(i,:), obj.arcY(i,:)] = obj.arcSegments(i).getXY();
                obj.centers(i,:) = obj.arcSegments(i).center;
           end
           
           curvature = obj.final_curv;
           radius = abs(1/curvature);
           position = [obj.arcSegments(end).x_coor(end) obj.arcSegments(end).y_coor(end)];
           start_angle = obj.arcSegments(end).final_angle;
           arcLen = obj.length/(2*obj.order);
           obj.arcSegments(end + 1) = arcSegment(position,start_angle,radius,arcLen,sign(curvature)); %TODO
           [obj.arcX(end,:), obj.arcY(end,:)] = obj.arcSegments(end).getXY();
           obj.centers(end,:) = obj.arcSegments(end).center;

           obj.allX = zeros(1,(obj.order+1)*obj.numPointsPerSegment);
           obj.allY = zeros(1,(obj.order+1)*obj.numPointsPerSegment);
           for j = 1:(obj.order + 1)

                obj.allX( ((j-1)*obj.numPointsPerSegment+1):(j*obj.numPointsPerSegment) )  = obj.arcX(j,:);
                obj.allY( ((j-1)*obj.numPointsPerSegment+1):(j*obj.numPointsPerSegment) ) = obj.arcY(j,:);

           end
           obj.final_tangent = obj.arcSegments(end).final_angle;
       end

       function plotClothoidWithCircles(obj)
            % There should be a better way to do this %TODO
            c = get(groot,'defaultAxesColorOrder');
            lenc = length(c);
            figure;
            for j = 1:(obj.order + 1)
                color = c(mod(j,lenc)+1,:);
                plot(obj.arcX(j,:), obj.arcY(j,:), 'Color',color);
                hold on
                if(obj.centers(j,1) ~= -1)
                    plot(obj.centers(j,1), obj.centers(j,2),'*', 'Color',color)
                    hold on
                    plot([obj.centers(j,1) obj.arcX(j,1)], ...
                        [obj.centers(j,2) obj.arcY(j,1)],'--', 'Color',color)
                    hold on 
                    plot([obj.centers(j,1) obj.arcX(j,end)], ...
                    [obj.centers(j,2) obj.arcY(j,end)],'--', 'Color',color)
                end
                hold on
            end

            title('Clothoid Path')
            xlabel('x')
            ylabel('y')
            grid on
       end
       function plotPlain(obj)
            % figure;

            plot(obj.allX,obj.allY,'LineWidth',1.5);
            % title('Clothoid Path')
            % xlabel('x')
            % ylabel('y')
            % grid on
       end
   end
end