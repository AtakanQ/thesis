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
    end
    
    methods
        function obj = bezier(A,B,tzero,tfinal,curvature_zero,curvature_final)
            
            %BEZIER Construct an instance of this class
            % tzero and tfinal are unit vectors
            tzero = tzero ./ norm(tzero);
            tfinal = tfinal ./ norm(tfinal);
            obj.Nt = 10;
            obj.Nk = 3;
            obj.mtmin = 0.3;
            obj.mtmax = 1.7;
            obj.mkmin = 0;
            obj.mkmax = 10;
            obj.Curves = cell(obj.Nt * obj.Nt * obj.Nk * obj.Nk, 1);
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
            rot_ccw = [0 1; -1 0];
            %create an array to easily loop
            acceleration_multiplier = linspace(obj.mkmin,obj.mkmax,obj.Nk);
            acceleration_tangential_multiplier = acceleration_multiplier * obj.dof;
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
                    curvature_zero * ( norm(tzero) )^2 * normal_tzero; %TODO
            
                obj.acceleration_final(n,:) = acceleration_tangential_multiplier(n) * tfinal +...
                    curvature_final * ( norm(tfinal) )^2 * normal_tfinal; %TODO
            end

        end
        
        function generateCurves(obj,A,B)
            P_zero = A;
            P_five = B;
            curve_position = 1;
            for tan_zero = 1:obj.Nt
                for tan_final = 1:obj.Nt
                    for acc_zero = 1:obj.Nk
                        for acc_final = 1:obj.Nk
                            P_one = P_zero + obj.tzero_vectors(tan_zero,:)/5;
            
                            P_two = obj.acceleration_zero(acc_zero,:)/20 + 2*P_one;
            
                            P_four = P_five - obj.tfinal_vectors(tan_final,:)/5;
            
                            P_three = obj.acceleration_final(acc_final,:)/20 + 2*P_four - P_five;
                            obj.Curves{tan_zero + tan_final + acc_zero+ acc_final - 3} = zeros(101,2);
                            pos = 1;
                            for t = 0:0.01:1
                                 obj.Curves{curve_position}(pos,:) =...
                                     (1-t)^5 *P_zero + 5*t*(1-t)^4*P_one +...
                                      10* t^2 *(1-t)^3 *P_two + 10 * t^3*(1-t)^2 *P_three+...
                                     +5*t^4*(1-t)*P_four + t^5*P_five;
                                 pos = pos +1;
                            end
            
                            curve_position = curve_position+1;
                        end
                    end
                end
            end

        end
        
        function plotCurves(obj)

            figure;
            for i = 1:numel(obj.Curves)
                plot(obj.Curves{i}(:,1),obj.Curves{i}(:,2))
                hold on
            end
            plot(obj.start(1),obj.start(2),'*','MarkerSize',10,'LineWidth',1.5,'Color',[0 0 1]);
            hold on
            plot(obj.finish(1),obj.finish(2),'*','MarkerSize',10,'LineWidth',1.5,'Color',[1 0 0]);
            axis equal
            title("Generated Beziér Curves")
            xlabel(" Distance (m) ")
            ylabel(" Distance (m) ")
        end
    end
end

