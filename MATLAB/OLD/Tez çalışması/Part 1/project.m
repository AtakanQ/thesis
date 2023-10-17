close all;
clear;
% states = [x y orientation curvature];
%% ROAD CORRIDOR EQUATION
sample_x = .8:0.05:5;
sample_y = sample_x.^3.*sin(sample_x);

goal = [sample_x(end) sample_y(end) 0 0];

plot(sample_x,sample_y,'Color',[1 0 0]);
hold on
plot(goal(1),goal(2),'d','Color',[0 1 0]);
title('Road Corridor Sample');
legend('Road Curve','Location','northeast');
xlim([0.5 6])
ylim([-130 20])
%% Quintic Beziér Curve Generation
% simulation velocity parameters
v_initial = [0 0 ];
v_final = [sqrt(2)/2 sqrt(2)/2];


% Assume Douglas Pecker Algorithm is applied already.
initial = [sample_x(1) sample_y(1) pi/4 0];
Nt = 10; % Tangent vector evaluation number
mtmin = 0.3;
mtmax = 1.7;


tangent_multiplier = linspace(mtmin,mtmax,Nt);

Nk = 3; % Curvature magnitude evaluation number
mkmin = 0;
mkmax = 10;

%first demonstrate first 15 points
line_x = sample_x(1:15);
line_y = sample_y(1:15);
figure;
plot(line_x,line_y);
xlim([0.5 6])
ylim([-130 20])

num_line_segments = (length(line_x) - 1);

% Create control points' structure array
% for i = 1:num_line_segments
%     %calculate tzero no normalization
%     if i == 1
%         %or maybe do nothing here
%         tzero = v_initial;
%         control_points(i).tzero = v_initial;
%     else
%         %t0 = Si - Si-1
%         tzero = [(line_x(i)-line_x(i-1)) (line_y(i)-line_y(i-1))];
%         %normalize tzero
%         tzero = tzero ./ norm(tzero);
% 
%         distance_zero = norm([line_x(i)-line_x(i-1) line_y(i)-line_y(i-1)]);
%     end
% 
%     if i == num_line_segments
%         %or maybe do nothing here
%         tfinal = v_final;
%         control_points(i).tfinal = v_final;
%     else
%         %t0 = Si - Si-1
%         tfinal = [(line_x(i+1)-line_x(i)) (line_y(i+1)-line_y(i))];
%         tfinal = tfinal ./ norm(tfinal);
% 
%         distance_final = norm([line_x(i+1)-line_x(i) line_y(i+1)-line_y(i)]);
%     end
% 
%     %Create Nt varying size of velocity vectors.
%     for m = 1:Nt
%         if i ~=1
%             control_points(i).tzero(m,:) = tzero .* tangent_multiplier(m) * distance_zero;
%         end
%         
%         if i ~= num_line_segments
%             control_points(i).tfinal(m,:) = tfinal .* tangent_multiplier(m) * distance_final;
%         end
%     end
% 
% 
% end

%Calculate first derivate at each point
rot_cw = [0 -1; 1 0];
rot_ccw = [0 1; -1 0];
num_of_points = length(line_x);
points = [line_x' line_y']; % 15x2 coordinates
tangents = zeros(num_line_segments+1 , 2);
for i = 1:(num_of_points)
    if i == 1
        continue
    elseif i == num_of_points
        tangents(i,:) = tangents(i-1,:);
    else % the tangent to be calculated is not at any extreme
        va = points(i,:) - points(i-1,:);
        vb = points(i+1,:) - points(i,:);
        va_angle = mod(atan2d(va(2) , va(1)) , 360);
        vb_angle = mod(atan2d(vb(2) , vb(1)) , 360);

        bisector = norm(vb) * va + norm(va) * vb;
        
        if( (va_angle - vb_angle) > 0)
            %rot_cw
            tangent_direction = bisector * rot_cw';
        else
            %rot_ccw
            tangent_direction = bisector * rot_ccw';
        end
        % normalize the vector
        tangents(i,:) = tangent_direction ./ norm(tangent_direction);
    end
end
tangents(1,:) = tangents(2,:);

%Find the curvatures




% plot the segment
% x = 0:0.01:1;
% curve_segment = (1-x).^5 * control_points(1).location + 5.*x.*(1-x).^4 * control_points(2).location...
%     + 10*x.^2.*(1-x).^3 * control_points(3).location + 10*x.^3.*(1-x).^2 * control_points(4).location...
%     +5*x^4.*(1-x).*control_points(5).location + x.^5.*control_points(6).location; 

