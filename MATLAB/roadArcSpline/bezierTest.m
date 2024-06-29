A = [1 1];
B = [10 3];
tzero = [cosd(45); sind(45)];
tfinal = [cosd(0); sind(0)];
tzero = (B-A)';
tfinal = (A-B)';
curvature_zero = 0.1;
curvature_final = 0;


myBezier = bezier(A,B,tzero, tfinal,curvature_zero,curvature_final);

myBezier.plotCurves()


