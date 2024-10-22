classdef clothoid_v2 < handle
   properties
      init_pos
      init_tan
      final_tan
      init_curv
      final_curv
      curv_length
      curv_sign
      allX
      allY
      numPoints
      allTangent
      allCurvature
      curv_derivative
   end

   methods
       function obj = clothoid_v2( ...
               init_pos, ...
               init_tan, ...
               init_curvature, ...
               final_curvature,...
               curv_length, ...
               dataPointSparsity ...
               )

         if nargin < 6
             % obj.numPoints = numPoints;
            obj.numPoints = ceil(curv_length / 0.1); % data point for each 10 centimeter
            obj.init_pos = init_pos;
            obj.init_tan = init_tan;
            obj.init_curv = init_curvature;
            obj.final_curv = final_curvature;
            obj.curv_length = curv_length;
            obj.curv_sign = sign(final_curvature - init_curvature);
            obj.curv_derivative = (final_curvature - init_curvature)/curv_length;
            obj.generateClothoid();

            if(obj.curv_sign == 0)
                error('Curvature is zero')
            end
         elseif nargin == 6
            obj.numPoints = ceil(curv_length / dataPointSparsity); % data point for each 10 centimeter
            obj.init_pos = init_pos;
            obj.init_tan = init_tan;
            obj.init_curv = init_curvature;
            obj.final_curv = final_curvature;
            obj.curv_length = curv_length;
            obj.curv_sign = sign(final_curvature - init_curvature);
            obj.curv_derivative = (final_curvature - init_curvature)/curv_length;
            obj.generateClothoid();

            if(obj.curv_sign == 0)
                error('Curvature is zero')
            end

         end
       end

       function generateClothoid(obj)
        % Arc length parameter along the clothoid
        s = linspace(0, obj.curv_length, obj.numPoints);
    
        % Numerical integration to calculate position, tangent, and curvature
        [obj.allX, obj.allY,obj.allTangent , obj.allCurvature] = ...
        obj.clothoidIntegration(s, ...
        obj.init_pos, ...
        obj.init_tan, ...
        obj.init_curv, ...
        obj.final_curv);
    

       end

    function [x, y, tangent, curvature] = clothoidIntegration(obj,s, initialPos, initialTangent, initialCurvature, finalCurvature)
        % Initialize arrays
        x = zeros(size(s));
        y = zeros(size(s));
        tangent = zeros(size(s));
        curvature = zeros(size(s));
    
        % Integration step size
        ds = s(2) - s(1);
    
        % Initial conditions
        x(1) = initialPos(1);
        y(1) = initialPos(2);
        tangent(1) = initialTangent;
        curvature(1) = initialCurvature;
    
        % Numerical integration using the clothoid equations
        for i = 2:numel(s)
            curvature(i) = curvature(i - 1) + ds * (finalCurvature - initialCurvature) / obj.curv_length;
            tangent(i) = tangent(i - 1) + ds * curvature(i - 1);
            x(i) = x(i - 1) + ds * cos(tangent(i - 1)) ;
            y(i) = y(i - 1) + ds * sin(tangent(i - 1)) ;
        end
        obj.final_tan = tangent(end);
    end

    function p1 = plotPlain(obj,color,legend)
            % figure;
            if (nargin == 1 )
                color = [0 0 1];
                legend = 'Integrated Clothoid';
            elseif(nargin == 2)
                legend = 'Integrated Clothoid';
            end
            p1 = plot(obj.allX,obj.allY,'LineWidth',1.5,'Color',color,'DisplayName',legend);
            axis equal
            % title('Clothoid Path')
            % xlabel('x')
            % ylabel('y')
            % grid on
       end
   end
end