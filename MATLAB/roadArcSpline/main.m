close all
clear
%% Define points

pts = [5 5; 25 30; 55 55; 110 100;120 115; 125 130; 135 160];

myRadii = findRadii(pts);


%% Define circular points

theta = 0:0.01:2*pi;
radius = 600;
x = radius*cos(theta);
y = radius*sin(theta);
figure;
plot(x,y)
title("Generated Sample Circle")
myRadii = findRadii([x ;y]');

disp("Radius of the sample circle:")
disp(num2str(1/myRadii(5), 100));

% for i = 1:length(myRadii)
%     disp(num2str(1/myRadii(i), 100));
% end

%% Define Beziér Curve
P0 = [0, 0];
P1 = [10, 15];
P2 = [15, 25];
P3 = [25, 20];
P4 = [40, 10];
P5 = [55, 90];
[x,y] = generateBezier(P0,P1,P2,P3,P4,P5);
figure;
plot(x,y)
title("Generated Sample Bezier Curve")
% plot(pts(:,1), pts(:,2))
% hold on
% plot(pts(:,1), pts(:,2),'*')

% for i = 2: (length(pts) - 1)
%     %Calculate AB
%     AB = pts(i - 1,:) - pts(i,:);
%     AB = [AB 0];
%     normAB = norm( AB );
% 
%     %Calculate AB
%     AC = pts(i + 1,:) - pts(i,:); 
%     AC = [AC 0]; 
%     normAC = norm( AC );
% 
%     %Calculate angle
%     small_angle = atan2(norm(cross(AB,AC)),dot(AB,AC));
% 
%     %Calculate D
%     norm_D = normAB * normAC * sin(small_angle);
%     D = [0 0 norm_D];
% 
%     %Calculate D
%     E = cross(D,AB);
%     norm_E = norm(E);
% 
%     %Calculate F
%     F = cross(D,AC);
%     norm_F = norm(F);
% 
%     e = E / norm_E;
%     f = F / norm_F;
% 
%     rho = ( normAC*normAC * E - normAB*normAB * F) / (2 * norm_D * norm_D);
%     1/norm(rho);
% end