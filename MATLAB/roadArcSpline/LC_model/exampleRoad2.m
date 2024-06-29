Sall = 0;
CircleData = [];
LineData = [];
n = 5;
plotOn = 1;
% ============================
% Straight
% ============================
figure;
hold all;
S = 300;
SP = [0;0];
thetaS = 0;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData LineDataNew];
PS = LineData(3:4,end);
thetaS = 0;
% ===========================
% First turn
% ===========================
% Discrete Clothoid
S = 200;
Sall = Sall+S;
kA = 0;
kB = .005;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
% Arc
S = 200; 
Sall = Sall+S;
ArcData = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData ArcData];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% Discrete Clothoid
S = 100;
Sall = Sall+S;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% ============================
% Straight
% ============================
S = 1000;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData LineDataNew];
PS = LineData(3:4,end);
% ============================
% Second turn
% ============================
% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 8;
kA = 0;
kB = -.005;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(4,end); % negative curvature
% Arc
S = 200;
Sall = Sall+S;
ArcData = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData ArcData];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 8;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(4,end);
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% ==============================
% Straight
% ==============================
S = 500;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData LineDataNew];
PS = LineData(3:4,end);
% ================================
% Third turn
% ================================
% Discrete Clothoid
S = 100;
Sall = Sall+S;
n = 5;
kA = 0;
kB = -.01;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end); 
Rend = CircleData(3,end);
thetaS = CircleData(4,end); % negative
% Arc
S = 100;
Sall = Sall+S;
ArcData = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData ArcData];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 8;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(4,end);
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% ==============================
% Straight
% ==============================
S = 1000;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData LineDataNew];
PS = LineData(3:4,end);
% =======================================
% Fourth Turn
% =======================================
S = 50;
Sall = Sall+S;
n = 4;
kA = 0;
kB = .008;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
% Arc
S = 500;
Sall = Sall+S;
ArcData = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData ArcData];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% Discrete Clothoid
S = 250;
Sall = Sall+S;
n = 5;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData CircleDataNew];
LineData = [LineData LineDataNew];
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
% ==============================
% Straight
% ==============================
S = 600;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData LineDataNew];
tSim = Sall/(v+2);