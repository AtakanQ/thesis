%% Try to fit clothoid
% load('bezierData.mat')
clear
load('../all_clothoids.mat')
posError =  +0.2; %meters in x direction
headingError = deg2rad(5); %radians
curvatureError = -0.01;

myClothoid = all_clothoids{1}(158);

vehicleX(1) = myClothoid.allX(1)+posError*cos(myClothoid.init_tan + pi/2);
vehicleY(1) = myClothoid.allY(1)+posError*sin(myClothoid.init_tan + pi/2);
vehicleHeading(1) = myClothoid.init_tan+headingError;
vehicleCurvature(1) = myClothoid.init_curv + curvatureError;

k0 = vehicleCurvature(1) - myClothoid.init_curv;
theta0 = vehicleHeading(1) - myClothoid.init_tan; 

l = -2*theta0/k0; 
sigma = k0^2/2/theta0; 

% Position error
deltaY = posError; 
lambda = 0.5; 
gamma = 0.5; 
[alpha,k1] = computeAlphak1(l,lambda,gamma,-deltaY,0); 
[CircleData,LineData,curvArc,curvVec,EP,theta_end,xVec,yVec] = computeBiElementaryPath(l,lambda,gamma,5,k1,[0;0],0,0); 


%Add S1 and A1 here
S1 = l/16*(2*k0 + sigma*l/8);
A1 = k1*l/16;
k_1 = vehicleCurvature + S1 + A1;

dummy_arcSeg= arcSegment;
clothoids(1) = clothoid([vehicleX(1) vehicleY(1)], vehicleHeading(1)...
    , vehicleCurvature(1), k_1, l/8 , 5, dummy_arcSeg); 

vehicleX(2) = clothoids(1).allX(end);
vehicleY(2) = clothoids(1).allY(end);
vehicleHeading(2) = clothoids(1).final_tangent;
vehicleCurvature(2) = clothoids(1).final_curv;

%Add S2 and A2 here
S2 = l/16*(2*k0 + 3*sigma*l/8);
A2 = k1*l/16;
k_2 = vehicleCurvature(1) + S2 + A2;

clothoids(2) = clothoid([vehicleX(2) vehicleY(2)], vehicleHeading(2)...
    , vehicleCurvature(2), k_2, l/8 , 5, dummy_arcSeg); 

vehicleX(3) = clothoids(2).allX(end);
vehicleY(3) = clothoids(2).allY(end);
vehicleHeading(3) = clothoids(2).final_tangent;
vehicleCurvature(3) = clothoids(2).final_curv;

%Add S3 and A3 here
k_3 = vehicleCurvature(1) + l/4*(2*k0 + sigma*l);

clothoids(3) = clothoid([vehicleX(3) vehicleY(3)], vehicleHeading(3)...
    , vehicleCurvature(3), k_3, l/2 , 5, dummy_arcSeg); 

vehicleX(4) = clothoids(3).allX(end);
vehicleY(4) = clothoids(3).allY(end);
vehicleHeading(4) = clothoids(3).final_tangent;
vehicleCurvature(4) = clothoids(3).final_curv;

%Add S4 and A4 here
k_4 = vehicleCurvature(1) + l/16*(2*k0 + 13*sigma*l/8) - k1*l/16;

clothoids(4) = clothoid([vehicleX(4) vehicleY(4)], vehicleHeading(4)...
    , vehicleCurvature(4), k_4, l/8 , 5, dummy_arcSeg); 

vehicleX(5) = clothoids(4).allX(end);
vehicleY(5) = clothoids(4).allY(end);
vehicleHeading(5) = clothoids(4).final_tangent;
vehicleCurvature(5) = clothoids(4).final_curv;

%Add S5 and A5 here
k_5 = vehicleCurvature(1) + l/16*(k0 + 7*sigma*l/8) - k1*l/16;

clothoids(5) = clothoid([vehicleX(5) vehicleY(5)], vehicleHeading(5)...
    , vehicleCurvature(5), k_5, l/8 , 5, dummy_arcSeg); 

vehicleX(6) = clothoids(5).allX(end);
vehicleY(6) = clothoids(5).allY(end);
vehicleHeading(6) = clothoids(5).final_tangent;
vehicleCurvature(6) = clothoids(5).final_curv;


figure
plot(myClothoid.allX,myClothoid.allY,'LineWidth',2,'DisplayName',"Ground truth",'Color',[0 0 1])
axis equal
hold on
for i = 1:numel(clothoids)
    plot(clothoids(i).allX,clothoids(i).allY,'LineWidth',1.5,'DisplayName',"Clothoid: "+ num2str(i))
end
legend()

figure;
plot(vehicleCurvature)

[clothoidArray] = fitArcSpline([vehicleX vehicleY],...
    vehicleHeading(1),vehicleCurvature(1),myClothoid);

%%
k0 = -0.01; 
% k0 = -0.1; 
theta0 = 5/180*pi; 

l = -2*theta0/k0; 
sigma = k0^2/2/theta0; 

arc_length = unique([0:.1:l l]);
k = k0 + sigma*arc_length; 
theta = theta0 + k0*arc_length + sigma/2*arc_length.^2; 

dummy_arcSeg= arcSegment;
clo = clothoid([0 0], theta0, k0, 0, l, 5, dummy_arcSeg); 

% Position error
deltaY = clo.allY(end); 
lambda = 0.5; 
gamma = 0.5; 
[alpha,k1] = computeAlphak1(l,lambda,gamma,-deltaY,0); 
[CircleData,LineData,curvArc,curvVec,EP,theta_end,xVec,yVec] = computeBiElementaryPath(l,lambda,gamma,5,k1,[0;0],0,0); 

[clo_unique_x, idx] = unique(clo.allX);
clo_unique_y = clo.allY(idx); 

[xVec,idx] = unique(xVec); 
yVec = yVec(idx);   

all_x = [0:.01:l l]; 
clo_y = interp1(clo_unique_x,clo_unique_y,all_x); 
bi_y = interp1(xVec,yVec,all_x); 

figure
subplot(3,1,1)
plot(arc_length,theta/pi*180,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('angle [°]')
subplot(3,1,2)
plot(arc_length,k,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('curvature [1/m]')
subplot(3,1,3)
plot(clo.allX,clo.allY,'LineWidth',2)
hold all; 
plot(xVec,yVec,'LineWidth',2)
plot(all_x,clo_y+bi_y,'LineWidth',2)
grid; 
xlabel('x-position [m]')
ylabel('y-position [m]')
%%
% =============================================
% Case 2
% =============================================
k0 = 0.005; 
theta0 = 5/180*pi; 
sigma = 0.005; 

l1 = k0/sigma + sqrt(k0^2/2/sigma^2 + theta0/sigma);  
l2 = l1 - k0/sigma; 
L = l1 + l2; 

arc_length1 = unique([0:0.1:l1 l1]);
arc_length2 = unique([0:0.1:l2 l2]);

k1 = k0 -sigma*arc_length1; 
k_2 = k1(end) + sigma*arc_length2; 

theta1 = theta0 + k0*arc_length1 - sigma/2*arc_length1.^2; 
theta2 = theta1(end) + k1(end)*arc_length2 + sigma/2*arc_length2.^2; 

clo1 = clothoid([0 0], theta0, k0, k0-sigma*l1, l1, 5, dummy_arcSeg); 
clo2 = clothoid([clo1.allX(end) clo1.allY(end)], clo1.final_tangent, k0-sigma*l1, 0, l2, 5, dummy_arcSeg); 

% Position error
deltaY = clo2.allY(end); 
lambda = 0.5; 
gamma = 0.5; 
[alpha,ka] = computeAlphak1(L,lambda,gamma,-deltaY,0); 
[CircleData,LineData,curvArc,curvVec,EP,theta_end,xVec,yVec] = computeBiElementaryPath(L,lambda,gamma,5,ka,[0;0],0,0); 

[clo_unique_x, idx] = unique([clo1.allX clo2.allX]);
allY = [clo1.allY clo2.allY]; 
clo_unique_y = allY(idx); 

[xVec,idx] = unique(xVec); 
yVec = yVec(idx); 

all_x = [0:.01:xVec(end) xVec(end)]; 
clo_y = interp1(clo_unique_x,clo_unique_y,all_x); 
bi_y = interp1(xVec,yVec,all_x); 


figure
subplot(3,1,1)
plot(arc_length1,theta1/pi*180,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,theta2/pi*180,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('angle [°]')
subplot(3,1,2)
plot(arc_length1,k1,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,k_2,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('curvature [1/m]')
subplot(3,1,3)
plot(all_x,clo_y,'LineWidth',2)
hold on; 
plot(all_x,bi_y,'LineWidth',2)
plot(all_x,clo_y+bi_y,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('curvature [1/m]')
