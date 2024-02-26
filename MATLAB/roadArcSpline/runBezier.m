% B0 = [0;0]; 
% B5 = [100;20];
% t1 = [10;0];
% t2 = [20;0];
% k1 = 0.01; 
% k2 = 0.0; 
% 
% figure
% hold all; 
% for kk = 1:20
%     t1 = [5*kk;0];
%     t2 = [5*kk;0]; 
%     [B1,B2,B3,B4] = quinticBezier_computation(B0,B5,t1,t2,k1,k2); 
%     quinticBezier(B0,B1,B2,B3,B4,B5,100); 
% end

B0 = [1;1]; 
B5 = [10;3];
t1_orig = [cosd(45);sind(45)];
t2_orig = [cosd(20);sind(20)];
k1 = 0.3; 
k2 = 0.0; 

% figure
% hold all; 
% for kk = 1:20
%     t1 = kk*t1_orig;
%     t2 = kk*t2_orig; 
%     [B1,B2,B3,B4] = quinticBezier_computation(B0,B5,t1,t2,k1,k2); 
%     quinticBezier(B0,B1,B2,B3,B4,B5,100); 
% end

myBezier = bezier(B0',B5',t1_orig,t2_orig,k1,k2);
myBezier.plotCurves()