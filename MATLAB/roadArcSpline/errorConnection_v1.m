k0 = -0.01; 
% k0 = -0.1; 
theta0 = 10/180*pi; 

l = -2*theta0/k0; 
sigma = k0^2/2/theta0; 

arc_length = unique([0:.1:l l]);
k = k0 + sigma*arc_length; 
theta = theta0 + k0*arc_length + sigma/2*arc_length.^2; 

figure
subplot(2,1,1)
plot(arc_length,theta/pi*180,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('angle [°]')
subplot(2,1,2)
plot(arc_length,k,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('curvature [1/m]')

k0 = 0.01; 
theta0 = 5/180*pi; 
theta = 0.001; 
L = 500; 

l1 = sqrt((k0^2 + 2*sigma*theta0 + 2*sigma*k0*L)/2/sigma^2);
l1 = k0/sigma + sqrt(k0^2/2/sigma^2 + theta0/sigma);  
l2 = l1 - k0/sigma; 
L = l1 + l2; 

arc_length1 = unique([0:0.1:l1 l1]);
arc_length2 = unique([0:0.1:l2 l2]);

k1 = k0 -sigma*arc_length1; 
k2 = k1(end) + sigma*arc_length2; 

theta1 = theta0 + k0*arc_length1 - sigma/2*arc_length1.^2; 
theta2 = theta1(end) + k1(end)*arc_length2 + sigma/2*arc_length2.^2; 

figure
subplot(2,1,1)
plot(arc_length1,theta1/pi*180,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,theta2/pi*180,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('angle [°]')
subplot(2,1,2)
plot(arc_length1,k1,'LineWidth',2)
hold on; 
plot(arc_length1(end)+arc_length2,k2,'LineWidth',2)
grid; 
xlabel('arc length [m]')
ylabel('curvature [1/m]')
