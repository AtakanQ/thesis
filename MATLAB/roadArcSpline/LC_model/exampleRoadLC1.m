Sall = 0;
CircleData = [];
LineData = [];
pointCount = 1;
% ============================
% Straight
% ============================
S = 50;
Sall = Sall+S;
SP = [0;0];
thetaS = 0;
plotOn = 0;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);

PS = LineData(3:4,end);
thetaS = 0;


k1 = k1Val;
lambda = lambdaVal;
gamma = 1;
kappa = 1;
 deltaY = 3.7;
[alpha,S] = computeAlphaS(k1,lambda,gamma,deltaY);
SLC = S;
Sall = Sall+S;
n = 5;

[CircleDataNew,LineDataNew] = clothoidLCComputation(S,k1,lambda,gamma,kappa,n,plotOn)
LineDataNew([1],:) = LineDataNew([1],:) + PS(1);
LineDataNew([3],:) = LineDataNew([3],:) + PS(1);
LineDataNew(end,:) = LineDataNew(end,:) + 1;
CircleDataNew(end,:) = CircleDataNew(end,:) + 1;
CircleDataNew(1,:) = CircleDataNew(1,:) + PS(1);
LineData = [LineData LineDataNew];
CircleData = [CircleData CircleDataNew];
pointCount = pointCount+size(LineDataNew,2)+size(CircleDataNew,2);

thetaS = CircleData(5,end);
SP = LineData(3:4,end);
% ============================
% Straight
% ============================
S = 500;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
Ts = 12;

[arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData);
plot(xVec,yVec);
hold all;

% tSim = Sall/(v+5); 
