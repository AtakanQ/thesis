function [CircleData,LineData] = clothoidLCComputation(S,k1,lambda,gamma,kappa,n,plotOn)
Sall = 0;
CircleData = [];
LineData = [];
pointCount = 1;
S1 = gamma*lambda*S;
S2 = gamma*(1-lambda)*S;
SL1 = kappa*(1-gamma)*S;
SL2 = (1-kappa)*(1-gamma)*S;
k2 = -lambda/(1-lambda)*k1;
PS = [0;0];
thetaS = 0;
% ===========================
% First turn
% ===========================
% Discrete Clothoid
S = S1/2;
Sall = Sall+S;
kA = 0;
kB = k1;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
if plotOn
    scatter(PS(1),PS(2))
end
% Discrete Clothoid
S = S1/2;
Sall = Sall+S;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = LineData(3:4,end);
if plotOn
    scatter(PS(1),PS(2));
end
% ============================
% Straight
% ============================
S = SL1;
Sall = Sall+S;
SP = PS;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
% ===========================
% Turn back
% ===========================
% Discrete Clothoid
S = S2/2;
Sall = Sall+S;
kA = 0;
kB = k2;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);

Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
if plotOn
    scatter(PS(1),PS(2))
end
% Discrete Clothoid
S = S2/2;
Sall = Sall+S;
kA = 1/Rend;
kB = .0;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);

CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
if plotOn
    scatter(PS(1),PS(2))
end

% ============================
% Straight
% ============================
S = SL2;
Sall = Sall+S;
SP = PS;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
 


