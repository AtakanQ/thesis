classdef bezier < handle
    %BEZIER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    % calculation parameters
    Nt
    Nk
    mtmin
    mtmax
    mkmin
    mkmax
    dof
    tzero_vectors
    tfinal_vectors
    acceleration_zero
    acceleration_final
    Curves
    start
    finish
    init_tan
    final_tan
    curv_zero
    curv_final
    P_one
    P_two
    P_three
    P_four
    isInLaneArray
    ltw
    lla
    llb
    vehicleBoundaries
    Tangents
    end
    
    methods
        function obj = bezier(A,B,tzero,tfinal,curvature_zero,curvature_final)
            
            %BEZIER Construct an instance of this class
            % tzero and tfinal are unit vectors
            tzero = tzero ./ norm(tzero);
            tfinal = tfinal ./ norm(tfinal);
            obj.init_tan = atan2(tzero(2),tzero(1));
            obj.final_tan = atan2(tfinal(2),tfinal(1));
            obj.curv_zero = curvature_zero;
            obj.curv_final = curvature_final;
            obj.Nt = 10;
            obj.Nk = 3;
            obj.mtmin = 0.3;
            obj.mtmax = 1.7;
            obj.mkmin = 0;
            obj.mkmax = 5;
            num_curves  = obj.Nt * obj.Nt * obj.Nk * obj.Nk;
            obj.Curves = cell(num_curves, 1);
            obj.Tangents = cell(num_curves, 1);
            obj.vehicleBoundaries.rearLeft = zeros(num_curves, 2);
            obj.vehicleBoundaries.rearRight = zeros(num_curves, 2);
            obj.vehicleBoundaries.frontLeft = zeros(num_curves, 2);
            obj.vehicleBoundaries.frontRight = zeros(num_curves, 2);
            obj.isInLaneArray = zeros(length(obj.Curves),1);
            obj.P_one = zeros(num_curves,2);
            obj.P_two = zeros(num_curves,2);
            obj.P_three = zeros(num_curves,2);
            obj.P_four = zeros(num_curves,2);

            obj.start = A;
            obj.finish = B;
            obj.dof = norm(A-B);% distance between initial and final point???
            obj.generateTangents(tzero,tfinal);
            obj.generateAccelerations(tzero,tfinal,curvature_zero,curvature_final);
            obj.generateCurves(A,B);
        end
        
        function generateTangents(obj,tzero,tfinal)

            obj.tzero_vectors = repmat(tzero',obj.Nt,1);
            obj.tfinal_vectors = repmat(tfinal',obj.Nt,1);
            
            tangent_multiplier = linspace(obj.mtmin,obj.mtmax,obj.Nt);

            % create tangent vectors according to given calculation
            for i = 1:obj.Nt
                obj.tzero_vectors(i,:) = obj.tzero_vectors(i,:)*tangent_multiplier(i)*obj.dof;
                obj.tfinal_vectors(i,:) = obj.tfinal_vectors(i,:)*tangent_multiplier(i)*obj.dof;
            end
        end

        function generateAccelerations(obj,tzero,tfinal,curvature_zero,curvature_final)
            % rotation matrices
            rot_cw = [0 -1; 1 0];
            % rot_ccw = [0 1; -1 0];
            %create an array to easily loop
            acceleration_multiplier = linspace(obj.mkmin,obj.mkmax,obj.Nk);
            acceleration_tangential_multiplier = acceleration_multiplier * obj.dof/5;
            %DOF IS TOO HIGH. MAKING IT SMALLER GIVES A MORE SENSIBLE ROUTE
            
            % for k = 1:obj.Nk
            %     acceleration_tangential_multiplier(k) = acceleration_multiplier(k) * obj.dof;
            % end

            

            obj.acceleration_zero = zeros(obj.Nk,2);
            obj.acceleration_final = zeros(obj.Nk,2);

            normal_tzero = sign(curvature_zero) * rot_cw * tzero;
            normal_tfinal = sign(curvature_final) * rot_cw * tfinal;

            for n = 1:obj.Nk
                obj.acceleration_zero(n,:) =  acceleration_tangential_multiplier(n) * tzero +...
                    obj.dof^2 * curvature_zero *  normal_tzero; %TODO
            
                obj.acceleration_final(n,:) = acceleration_tangential_multiplier(n) * tfinal +...
                    obj.dof^2 * curvature_final *  normal_tfinal; %TODO
            end

        end
        
        function generateCurves(obj,A,B)
            t = (0:0.001:1)';
            P_zero = A;
            P_five = B;
            curve_position = 1;
            for tan_zero = 1:obj.Nt
                for tan_final = 1:obj.Nt
                    for acc_zero = 1:obj.Nk
                        for acc_final = 1:obj.Nk
                            obj.P_one(curve_position,:) = P_zero + obj.tzero_vectors(tan_zero,:)/5;
            
                            obj.P_two(curve_position,:) = obj.acceleration_zero(acc_zero,:)/20 + 2*obj.P_one(curve_position,:) - P_zero;
            
                            obj.P_four(curve_position,:) = P_five - obj.tfinal_vectors(tan_final,:)/5;
            
                            obj.P_three(curve_position,:) = obj.acceleration_final(acc_final,:)/20 + 2*obj.P_four(curve_position,:) - P_five;
                            obj.Curves{curve_position} = zeros(1001,2);
                            obj.Tangents{curve_position} = zeros(1001,1);
                            % pos = 1;

                            obj.Curves{curve_position} =...
                                 (1-t).^5 *P_zero + 5*t.*(1-t).^4*obj.P_one(curve_position,:) +...
                                  10* t.^2 .*(1-t).^3 *obj.P_two(curve_position,:) + 10 * t.^3.*(1-t).^2 *obj.P_three(curve_position,:)+...
                                 +5*t.^4.*(1-t)*obj.P_four(curve_position,:) + t.^5*P_five;

                            % dX = 5 * t.^4 * (obj.P_one(curve_position,:) - P_zero) + ...
                            %         20 * t.^3 * (obj.P_two(curve_position,:) - 2 * obj.P_one(curve_position,:) + P_zero) + ...
                            %         30 * t.^2 * (obj.P_three(curve_position,:) - 3 * obj.P_two(curve_position,:) + 3 * obj.P_one(curve_position,:) - P_zero) + ...
                            %         20 * t * (obj.P_four(curve_position,:) - 4 * obj.P_three(curve_position,:) + 6 * obj.P_two(curve_position,:) - 4 * obj.P_one(curve_position,:) + P_zero) + ...
                            %          5 * ones(length(t),1) * (P_five - obj.P_four(curve_position,:) + obj.P_three(curve_position,:) - obj.P_two(curve_position,:) + obj.P_one(curve_position,:) - P_zero);
                            dx = diff(obj.Curves{curve_position}(:, 1));
                            dy = diff(obj.Curves{curve_position}(:, 2));
                            obj.Tangents{curve_position} = atan2(dy,dx);

                            % for t = 0:0.001:1
                            %     obj.Curves{curve_position}(pos,:) =...
                            %          (1-t)^5 *P_zero + 5*t*(1-t)^4*obj.P_one(curve_position,:) +...
                            %           10* t^2 *(1-t)^3 *obj.P_two(curve_position,:) + 10 * t^3*(1-t)^2 *obj.P_three(curve_position,:)+...
                            %          +5*t^4*(1-t)*obj.P_four(curve_position,:) + t^5*P_five;
                            % 
                            %     dX = 5 * (obj.P_one(curve_position,:) - P_zero) * t^4 + ...
                            %         20 * (obj.P_two(curve_position,:) - 2 * obj.P_one(curve_position,:) + P_zero) * t^3 + ...
                            %         30 * (obj.P_three(curve_position,:) - 3 * obj.P_two(curve_position,:) + 3 * obj.P_one(curve_position,:) - P_zero) * t^2 + ...
                            %         20 * (obj.P_four(curve_position,:) - 4 * obj.P_three(curve_position,:) + 6 * obj.P_two(curve_position,:) - 4 * obj.P_one(curve_position,:) + P_zero) * t + ...
                            %          5 * (P_five - obj.P_four(curve_position,:) + obj.P_three(curve_position,:) - obj.P_two(curve_position,:) + obj.P_one(curve_position,:) - P_zero);
                            % 
                            %     obj.Tangents{curve_position}(pos) = atan2(dX(2),dX(1));
                            % 
                            %     % Normalize the derivative vector to get the tangent vector
                            %     % tangent_vector = dX / norm(dX);
                            % 
                            % 
                            %     pos = pos +1;
                            % end
            
                            curve_position = curve_position+1;
                        end
                    end
                end
            end

        end
        
        function addVehicleDimensions(obj,ltw ,lla, llb)
            %The vehicle's position is assumed to be the center of rear
            %axle. dimensions = [ltw lla llb]
            obj.ltw = ltw;
            obj.lla = lla;
            obj.llb = llb;
        end
        

        function plotCurves(obj)

            figure;
            plot(obj.start(1),obj.start(2),'*','MarkerSize',10,'LineWidth',1.5,'Color',[0 0 1]);
            hold on
            plot(obj.finish(1),obj.finish(2),'*','MarkerSize',10,'LineWidth',1.5,'Color',[1 0 0]);
            axis equal
            title("Generated Beziér Curves")
            xlabel(" Distance (m) ")
            ylabel(" Distance (m) ")
            % ylim([0 6])
            % xlim([-1 12])
            for i = 1:numel(obj.Curves)
                if(mod(i,100) == 0)
                    color = floor(i/100) / 9;
                    plot(obj.Curves{i}(:,1),obj.Curves{i}(:,2),'Color',[0 color color])
                    hold on
                    % plot(obj.P_one(i,1),obj.P_one(i,2),'o')
                    % plot(obj.P_two(i,1),obj.P_two(i,2),'o')
                    % plot(obj.P_three(i,1),obj.P_three(i,2),'o')
                    % plot(obj.P_four(i,1),obj.P_four(i,2),'o')
                    disp('Debugging bezier')
                end

            end

            annotation('textbox', [0.2, 0.1, 0.1, 0.1], 'String', string( ['init tan:',num2str(rad2deg(obj.init_tan)),...
                '  init curv:',num2str(obj.curv_zero),'  final tan:',num2str(rad2deg(obj.final_tan)),'  final curv:',num2str(obj.curv_final) ] ),'FitBoxToText','on')

        end

        function isInLane(obj,segment)
            % segments data points should not be sparse. It should be as
            % frequent as possible
            sz = size(segment.allX);
            
            for i = 1:length(obj.Curves)
                if sz(1) == 1
                    [rms_error, max_error, errors] = computeSegmentError(obj.Curves{i},[segment.allX' segment.allY']);
                elseif sz(2) == 1
                    [rms_error, max_error, errors] = computeSegmentError(obj.Curves{i},[segment.allX segment.allY]);
                else
                    disp('There is some sort of error with dimensions')
                end

                if(sum(errors > 1.8) > 0) %There is some curve that is far from lane center
                    obj.isInLaneArray(i) = 0;
                else
                    obj.isInLaneArray(i) = 1;
                end
            end
        end
    end
end

