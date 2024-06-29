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
k2 = k1(end) + sigma*arc_length2; 

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
plot(arc_length1(end)+arc_length2,k2,'LineWidth',2)
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
