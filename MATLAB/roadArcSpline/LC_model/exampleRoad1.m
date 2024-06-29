Sall = 0;
SLC = 0;
CircleData = [];
LineData = [];
plotOn = 1;
pointCount = 1;
curvVec = [];
curvArc = [];
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
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
thetaS = 0;
curvArc = [0:1:Sall];
curvVec = zeros(size(curvArc));

% ===========================
% First turn
% ===========================
% Discrete Clothoid
S = 100;
Sall = Sall+S;
n = 5;
kA = 0;
kB = .01;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Arc
S = 100; 
Sall = Sall+S;
CircleDataNew = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB*ones(size([Sall-S:1:Sall]) ) ];

% Discrete Clothoid
S = 100;
Sall = Sall+S;
n = 5;
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
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ============================
% Straight
% ============================
S = 300;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
curvArc = [curvArc Sall-S:1:Sall];
curvVec = [curvVec zeros(size(Sall-S:1:Sall))];

% ============================
% Second turn
% ============================
% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 4;
kA = 0;
kB = -.005;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);

Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end); % negative curvature
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Arc
S = 200;
Sall = Sall+S;
CircleDataNew = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB*ones(size([Sall-S:1:Sall]) ) ];

% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 4;
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
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ==============================
% Straight
% ==============================
S = 500;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
curvArc = [curvArc Sall-S:1:Sall];
curvVec = [curvVec zeros(size(Sall-S:1:Sall))];

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
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);

Cend = CircleData(1:2,end); 
Rend = CircleData(3,end);
thetaS = CircleData(5,end); % negative
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Arc
S = 100;
Sall = Sall+S;
CircleDataNew = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB*ones(size([Sall-S:1:Sall]) ) ];

% Discrete Clothoid
S = 150;
Sall = Sall+S;
n = 5;
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
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ==============================
% Straight
% ==============================
S = 1000;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end);
curvArc = [curvArc Sall-S:1:Sall];
curvVec = [curvVec zeros(size(Sall-S:1:Sall))];

% =======================================
% Fourth Turn
% =======================================
S = 50;
Sall = Sall+S;
n = 3;
kA = 0;
kB = .008;
[CircleDataNew,LineDataNew] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);

Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Arc
S = 200;
Sall = Sall+S;
CircleDataNew = arcSegment(Cend,Rend,thetaS,S);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB*ones(size([Sall-S:1:Sall]) ) ];

% Discrete Clothoid
S = 250;
Sall = Sall+S;
n = 5;
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
SP = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ==============================
% Straight
% ==============================
S = 600;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
curvArc = [curvArc Sall-S:1:Sall];
curvVec = [curvVec zeros(size(Sall-S:1:Sall))];

% [arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData);
% plot(xVec,yVec,'LineWidth',2);
% xlabel('X-position [m]','FontSize',12)
% ylabel('Y-position [m]','FontSize',12)
% set(gca,'FontSize',12)


muVal = .82;
g = 9.81;

% Compute velocity profile
% Velocity at position 400 should be mu*g/k1
accGain = 1;
v0 = sqrt(muVal*g/.01-accGain*2*-1*400);
tSim = Sall/(v0+5);
acc01 = -1;
S01 = 400;
v1 = sqrt(muVal*g/.01);
acc1 = 0;
v2 = sqrt(muVal*g/.005);
S12 = 1050-500;
acc12 = ((v2^2-v1^2)/2/S12);
acc2 = 0;
S23 = 2000-1250;
v3 = sqrt(muVal*g/.01);
acc3 = 0;
acc23 = ((v3^2-v2^2)/2/S23);
S34 = 3300-2150;
v4 = sqrt(muVal*g/.008);
acc34 = ((v4^2-v3^2)/2/S34);
acc4 = .5;

% decouplingParameters_yL
decouplingParametersNested

 sim('laneKeepingArcSplinesNestedRoad_2013a.slx',124)
% sim('laneKeepingArcSplinesNested_omer.slx',124)

vmaxVec = sqrt(sqrt(muVal^2*g^2)./abs(curvVec));
figure(5);
subplot(2,1,1)
plot(curvArc,curvVec,'LineWidth',2)
xlabel('arc-length [m]','FontSize',12)
ylabel('road curvature $k_{\mathcal{T}}$','FontSize',12,'Interpreter','latex')
grid
subplot(2,1,2)
plot(curvArc,vmaxVec,'LineWidth',2)
xlabel('arc-length [m]','FontSize',12)
ylabel('velocities [m/sec]','FontSize',12)
grid
hold all;
plot(arclength1,vel1,'LineWidth',2)
lll = legend('$v_{\mathrm{max}}$','$\mathcal{V}(s)$')
set(lll,'Interpreter','latex')
set(gca,'Ylim',[20 50])

figure(6);
plot([0 arclength1(end)],[muVal*g muVal*g],'k--','LineWidth',2)
hold all;
plot(arclength1, abs(acc_long1),'LineWidth',2)
plot(arclength1, sqrt(acc_long1.^2+acc_lat1.^2 ),'LineWidth',2)
lll = legend('$\mu\cdot g$','$a_{long}$','$\sqrt{a_{long}\sp2 + a_{lat}\sp2}$')
set(lll,'FontSize',12,'Interpreter','latex')
xlabel('X-position [m]','FontSize',12)
ylabel('accelerations [m/sec^2]','FontSize',12)
grid on;

figure(1)
plot(Xcurve1,Ycurve1)

figure(7);
plot(time,yL1,'LineWidth',2)
xlabel('time [sec]','FontSize',12)
ylabel('Tracking error [m]','FontSize',12)
set(gca,'FontSize',12,'Xlim',[0 125])
grid