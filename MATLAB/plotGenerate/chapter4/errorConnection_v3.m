close all;
clear all; 
dummy_arcSeg= arcSegment;

% =============================================
% Case 1
% =============================================
k0 = -0.005; 
theta0 = 10/180*pi; 
y0 = 0;

sigma = k0*k0/(2*theta0);
L = -2*theta0/k0;

arc_length1 = unique([0:0.1:L L]);

kl1 = k0 + sigma*arc_length1; 

k = @(s) k0 + sigma * s; % Example function of s
theta_nonconstant = arrayfun(@(x) integral(k, 0, x), arc_length1);
theta_nonconstant = theta0 + theta_nonconstant;

figure

subplot(2,1,1)

plot(arc_length1,theta_nonconstant/pi*180,'LineWidth',2)
grid on; 
xlabel('arc length (m)',"FontSize",13)
ylabel('angle (°)',"FontSize",13)
title("Heading Change Over Arc Spline","FontSize",13)
subplot(2,1,2)

plot(arc_length1,kl1,'LineWidth',2)
grid on; 
xlabel('arc length (m)',"FontSize",13)
ylabel('curvature (m^-^1)',"FontSize",13)
title("Curvature Change Over Arc Spline","FontSize",13)
%% CAse 2
% =============================================
% Case 2
% =============================================
k0 = 0.005; 
theta0 = 10/180*pi; 
y0 = 0; 
sigma = 0.0025; 


l1 = k0/sigma + sqrt(k0^2/2/sigma^2 + theta0/sigma);  
l2 = l1 - k0/sigma; 
L = l1 + l2; 

arc_length1 = unique([0:0.1:l1 l1]);
arc_length2 = unique([0:0.1:l2 l2]);

kl1 = k0 -sigma*arc_length1; 
kl2 = kl1(end) + sigma*arc_length2; 

theta1 = theta0 + k0*arc_length1 - sigma/2*arc_length1.^2; 
theta2 = theta1(end) + kl1(end)*arc_length2 + sigma/2*arc_length2.^2; 

clo1 = clothoid([0 0], theta0, k0, k0-sigma*l1, l1, 5, dummy_arcSeg); 
clo2 = clothoid([clo1.allX(end) clo1.allY(end)], clo1.final_tangent, k0-sigma*l1, 0, l2, 5, dummy_arcSeg); 

% Position error
deltaY = clo2.allY(end); 
lambda = 0.5; 
gamma = 0.5; 
[alpha,k1] = computeAlphak1(L,lambda,gamma,-deltaY,0); 
[CircleData,LineData,curvArc,curvVec,EP,theta_end,xVec,yVec] = computeBiElementaryPath(L,lambda,gamma,5,k1,[0;0],0,0); 
SE1 = L*gamma*lambda; 
SE2 = L*gamma*(1-lambda); 
SL = L*(1-gamma); 

% Curvature slopes 
sigma1 = k1(end)/(SE1/2); 
k2 = -lambda/(1-lambda)*k1; 
sigma2 = k2(end)/(SE2/2); 
sigmal1 = -sigma; 
sigmal2 = sigma; 

curvatures = [0 k0; SE1/2 k0+k1(end)+sigmal1*SE1/2; SE1 k0+sigmal1*SE1; l1 k0+sigmal1*l1; ...
    SE1+SL k0+sigmal1*l1+sigmal2*(SE1+SL-l1); L-SE2/2 k0+sigmal1*l1+sigmal2*(L-SE2/2-l1)+sigma2*SE2/2; ...
    L k0+sigmal1*l1+sigmal2*l2]; 

sim('clothoidSequenceSimulation.slx',L); 


[clo_unique_x, idx] = unique([clo1.allX clo2.allX]);
allY = [clo1.allY clo2.allY]; 
clo_unique_y = allY(idx); 

[xVec,idx] = unique(xVec); 
yVec = yVec(idx); 

all_x = [0:.01:xVec(end) xVec(end)]; 
clo_y = interp1(clo_unique_x,clo_unique_y,all_x); 
bi_y = interp1(xVec,yVec,all_x); 


figure
subplot(2,1,1)
plot(arc_length1,theta1/pi*180,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,theta2/pi*180,'LineWidth',2)
grid on; 
xlabel('arc length (m)',"FontSize",13)
ylabel('angle (°)',"FontSize",13)
title("Heading Change Over Arc Spline","FontSize",13)
subplot(2,1,2)
plot(arc_length1,kl1,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,kl2,'LineWidth',2)
grid on; 
xlabel('arc length (m)',"FontSize",13)
ylabel('curvature (m^-^1)',"FontSize",13)
title("Curvature Change Over Arc Spline","FontSize",13)

%% Case 2 only curvature

figure;
plot(arc_length1,kl1,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,kl2,'LineWidth',2)
grid on; 
xlabel('arc length (m)',"FontSize",13)
ylabel('curvature (m^-^1)',"FontSize",13)
title("Curvature Change Over Arc Spline","FontSize",13)
set(gca, 'XTick', [0 arc_length1(end) arc_length1(end)+arc_length2(end)], 'YTick', [kl1(end)  kl2(end) k0]);
yticklabels({'k_i_n_t','0','k_i'})
xticklabels({'0','l_1','l_1+l_2'})
set(gca, 'FontSize', 13);