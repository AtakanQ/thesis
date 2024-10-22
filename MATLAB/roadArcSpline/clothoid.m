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
      curvatures
      curv_derivative
      arcLenArray
      cumulativeArcLenArray
      arcTangents
      computationTime
      dataPointSparsity
   end

   methods
       function obj = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               length,order,arcSegClass,dataPointSparsity)
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
            if nargin == 8
                obj.dataPointSparsity = dataPointSparsity;
                obj.numPointsPerSegment = ceil(length / dataPointSparsity) / order; % point for each 1 centimeter
            else
                obj.dataPointSparsity = 0.01;
                obj.numPointsPerSegment = ceil(length / 0.01) / order; % point for each 1 centimeter
            end
            obj.curv_derivative = (final_curvature - init_curvature)/length;
            obj.generateArcSegments();
            
            if(obj.curv_sign == 0)
                error('Curvature is zero')
            end
         end
       end

       function generateArcSegments(obj)
           tempTime = 0;
           obj.centers = zeros(obj.order+1,2);
           obj.curvatures = zeros(obj.order+1,1);
           obj.arcX = cell(obj.order+1,1);
           obj.arcY = cell(obj.order+1,1);
           obj.arcLenArray = zeros(obj.order + 1,1);
            obj.cumulativeArcLenArray = zeros(obj.order + 1,1);
            obj.arcTangents = zeros(obj.order + 1,1);
           obj.arcSegments(1) = arcSegment(obj.init_pos,obj.init_tan,...
               abs(inv(obj.init_curv)),obj.length/(2 * obj.order),sign(obj.init_curv),obj.numPointsPerSegment/2);
            tic
           [obj.arcX{1}, obj.arcY{1}] = obj.arcSegments(1).getXY();
            tempTime = toc + tempTime ;
           obj.centers(1,:) = obj.arcSegments(1).center;
           obj.curvatures(1) =  obj.init_curv;
           obj.arcLenArray(1) = obj.length/(2 * obj.order);
           obj.cumulativeArcLenArray(1) = obj.length/(2 * obj.order);
           obj.arcTangents(1) = obj.init_tan;
           for i = 2:(obj.order)
                curvature = obj.init_curv + (i-1) * (obj.final_curv - obj.init_curv) / obj.order;
                radius = abs(1/curvature);
                position = [obj.arcSegments(i-1).x_coor(end) obj.arcSegments(i-1).y_coor(end)];
                arcLen = obj.length/obj.order;
                start_angle = obj.arcSegments(i-1).final_angle;
                
                
                obj.arcSegments(i) = arcSegment(position,start_angle,radius,arcLen,sign(curvature),obj.numPointsPerSegment);
                tic
                [obj.arcX{i}, obj.arcY{i}] = obj.arcSegments(i).getXY();
                tempTime = toc + tempTime;

                obj.arcTangents(i) = start_angle;
                obj.cumulativeArcLenArray(i) = obj.cumulativeArcLenArray(i-1) + arcLen;
                obj.arcLenArray(i) = arcLen;
                obj.centers(i,:) = obj.arcSegments(i).center;   
                obj.curvatures(i) = curvature;
           end
           
            curvature = obj.final_curv;
            radius = abs(1/curvature);
            position = [obj.arcSegments(end).x_coor(end) obj.arcSegments(end).y_coor(end)];
            start_angle = obj.arcSegments(end).final_angle;
            arcLen = obj.length/(2*obj.order);
            obj.arcSegments(end + 1) = arcSegment(position,start_angle,radius,arcLen,sign(curvature),obj.numPointsPerSegment/2);
            tic
            [obj.arcX{end}, obj.arcY{end}] = obj.arcSegments(end).getXY();
            tempTime = toc + tempTime;
            obj.computationTime = tempTime;
            obj.centers(end,:) = obj.arcSegments(end).center;
            obj.curvatures(end) = curvature;
            obj.arcLenArray(end) = arcLen;
            obj.cumulativeArcLenArray(end) = obj.cumulativeArcLenArray(end-1) + arcLen;
            obj.arcTangents(end) = start_angle;
           % obj.allX = zeros(1,(obj.order+1)*obj.numPointsPerSegment);
           % obj.allY = zeros(1,(obj.order+1)*obj.numPointsPerSegment);
            obj.allX = [];
            obj.allY = [];

            for j = 1:(obj.order + 1)

            obj.allX  = [obj.allX obj.arcX{j}];
            obj.allY = [obj.allY obj.arcY{j}];

           end
           obj.final_tangent = obj.arcSegments(end).final_angle;
       end

       function plotClothoidWithCircles(obj)
            % There should be a better way to do this %TODO
            c = get(groot,'defaultAxesColorOrder');
            lenc = length(c);
            % figure;
            for j = 1:(obj.order + 1)
                color = c(mod(j,lenc)+1,:);
                plot(obj.arcX{j}, obj.arcY{j}, 'Color',color,'LineWidth',2.5);
                hold on
                if(obj.centers(j,1) ~= -1)
                    plot(obj.centers(j,1), obj.centers(j,2),'*', 'Color',color)
                    hold on
                    plot([obj.centers(j,1) obj.arcX{j}(1)], ...
                        [obj.centers(j,2) obj.arcY{j}(1)],'--', 'Color',color)
                    hold on 
                    plot([obj.centers(j,1) obj.arcX{j}(end)], ...
                    [obj.centers(j,2) obj.arcY{j}(end)],'--', 'Color',color)
                    text(obj.centers(j,1)*1.05,obj.centers(j,2)*1.05,"C_"+num2str(j),...
                        'HorizontalAlignment','left','VerticalAlignment','middle','FontSize',9 ,'Color',color)
                end
                hold on
            end

            % title('Clothoid Path')
            % xlabel('x')
            % ylabel('y')
            axis equal
            grid on
            ylim([0 12])
       end
       function p1 = plotPlain(obj,color,legend)
            % figure;
            if (nargin == 1 )
                color = [0 0 1];
                legend = 'Arc Approximated Clothoid';
            elseif(nargin == 2)
                legend = 'Arc Approximated Clothoid';
            end
            p1 = plot(obj.allX,obj.allY,'LineWidth',1.5,'DisplayName',legend,'Color',color);
            axis equal
            % title('Clothoid Path')
            % xlabel('x')
            % ylabel('y')
            % grid on
       end
   end
end