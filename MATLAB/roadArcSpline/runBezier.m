
B0 = [1;1]; 
B5 = [10;3];
t1_orig = [cosd(45);sind(45)];
t2_orig = [cosd(20);sind(20)];
k1 = 0.3; 
k2 = 0.0; 

myBezier = bezier(B0',B5',t1_orig,t2_orig,k1,k2);
myBezier.plotCurves()

B0 = [1;1]; 
B5 = [10;5];
t1_orig = [cosd(45);sind(45)];
t2_orig = [cosd(43);sind(43)];
k1 = -0.01; 
k2 = -0.01; 

myBezier = bezier(B0',B5',t1_orig,t2_orig,k1,k2);
myBezier.plotCurves()

B0 = [1;1]; 
B5 = [10;1];
t1_orig = [cosd(90);sind(90)];
t2_orig = [cosd(-90);sind(-90)];
k1 = 0.3; 
k2 = 0.0; 

myBezier = bezier(B0',B5',t1_orig,t2_orig,k1,k2);
myBezier.plotCurves()

