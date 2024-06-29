%% Trial for LC maneuver with traj data

% Load here LC_startf,S_numf,deltaYf,k1f,lambdaf

load LCdata_trial_13ms
% load LCdata_trial_20ms

%% Maneuver Data Entrance

v_ent = LCdata.v_ent;
v0_LC = LCdata.v0;
S_numf = LCdata.S_numf;
k1f = LCdata.k1f;
lambdaf = LCdata.lambdaf;
LC_startf = LCdata.LC_startf;

acc = (v_ent^2-v0_LC^2)/(2*LC_startf); % deleceration value

End = LCdata.End;
Start2 = LCdata.Start2;
deltaYf = LCdata.deltaYf;

deltaY2 = -deltaYf;
k1_mir = lambdaf/(1-lambdaf)*k1f*sign(deltaY2);
lambda_mir = 1-lambdaf;

n = 5; % discretization points
%% Trajectory Re-generation

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
S = LC_startf;
SP = [0;0];
thetaS = 0;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end); % should be LC_startf
thetaS = 0;
curvArc = [0:1:Sall];
curvVec = zeros(size(curvArc));

% ===========================
% First turn --- Lane Change
% ===========================
% Discrete Clothoid
S = (lambdaf*S_numf)/2; % S1/2 demek oluyor
Sall = Sall+S;
kA = 0;
kB = k1f;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Discrete Clothoid
S = (lambdaf*S_numf)/2;
Sall = Sall+S;
kA = k1f; % k1f olması gerekiyor
kB = 0;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
% PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
PS = LineData(3:4,end);
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ============================
% Second turn
% ============================
% Discrete Clothoid
S = ((1-lambdaf)*S_numf)/2; % S2/2 oluyor burası
Sall = Sall+S;
kA = 0;
kB = -k1f*lambdaf/(1-lambdaf); % k2 oluyor burası
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end); % negative curvature
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Discrete Clothoid
S = ((1-lambdaf)*S_numf)/2; % S2/2 oluyor burası
Sall = Sall+S;
kA = -k1f*lambdaf/(1-lambdaf); % k2 olması lazım buranın kontrol et
kB = 0;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
% PS = Cend + Rend*[sin(thetaS); -cos(thetaS)]; % Last point of LC maneuver
PS = LineData(3:4,end);
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ============================
% Straight
% ============================
hold all;
S = norm(Start2-End);
% SP = End;
SP = PS;
thetaS = 0;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end); % should be LC_startf
thetaS = 0;
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec zeros(size([Sall-S:1:Sall]))];

% ===========================
% First turn --- Recovery
% ===========================
% Discrete Clothoid
S = (lambda_mir*S_numf)/2; % S1/2 demek oluyor
Sall = Sall+S;
kA = 0;
kB = k1_mir;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];


% Discrete Clothoid
S = (lambda_mir*S_numf)/2;
Sall = Sall+S;
kA = k1_mir; % k1f olması gerekiyor
kB = .0;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
% PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
PS = LineData(3:4,end);
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ============================
% Second turn
% ============================
% Discrete Clothoid
S = ((1-lambda_mir)*S_numf)/2; % S2/2 oluyor burası
Sall = Sall+S;
kA = 0;
kB = -k1_mir*lambda_mir/(1-lambda_mir); % k2 oluyor burası
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end); % negative curvature
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kB/S*([Sall-S:1:Sall]-(Sall-S)) ];

% Discrete Clothoid
S = ((1-lambda_mir)*S_numf)/2; % S2/2 oluyor burası
Sall = Sall+S;
kA = -k1_mir*lambda_mir/(1-lambda_mir); % k2 olması lazım buranın kontrol et
kB = .0;
[CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
pointCount = pointCount+size(CircleDataNew,2);
LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
Cend = CircleData(1:2,end);
Rend = CircleData(3,end);
thetaS = CircleData(5,end);
PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec kA/S*(Sall-[Sall-S:1:Sall]) ];

% ============================
% Straight -- Last 30m straight line
% ============================
hold all;
S = 30;
% SP = End;
SP = PS;
thetaS = 0;
Sall = Sall+S;
[LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
pointCount = pointCount+size(LineDataNew,2);
PS = LineData(3:4,end); % should be LC_startf
thetaS = 0;
curvArc = [curvArc [Sall-S:1:Sall]];
curvVec = [curvVec zeros(size([Sall-S:1:Sall]))];

[arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData); % no need for arcVec
% plot(xVec,yVec,'LineWidth',2);
% xlabel('X-position [m]','FontSize',12)
% ylabel('Y-position [m]','FontSize',12)
% set(gca,'FontSize',12)

muVal = .82;
g = 9.81;


%% Simulation Time

% Compute velocity profile
v0 = v_ent;
tSim = Sall/(v0+2);
acc_sim = -acc;
S01 = LC_startf;
v1 = v0_LC;
acc1 = 0;

% decouplingParameters_yL
decouplingParametersNested

% sim('laneKeepingArcSplinesNestedRoad_2013a.slx',tSim)
sim('laneKeepingArcSplinesNested_omer.slx',tSim)


%% Plots

vmaxVec = sqrt(sqrt(muVal^2*g^2)./abs(curvVec));
figure;
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
lll = legend('$v_{\mathrm{max}}$','$\mathcal{V}(s)$');
set(lll,'Interpreter','latex')
% set(gca,'Ylim',[20 50])

figure;
plot([0 arclength1(end)],[muVal*g muVal*g],'k--','LineWidth',2)
hold all;
plot(arclength1, abs(acc_long1),'LineWidth',2)
plot(arclength1, sqrt(acc_long1.^2+acc_lat1.^2 ),'LineWidth',2)
lll = legend('$\mu\cdot g$','$a_{long}$','$\sqrt{a_{long}\sp2 + a_{lat}\sp2}$');
set(lll,'FontSize',12,'Interpreter','latex')
xlabel('X-position [m]','FontSize',12)
ylabel('accelerations [m/sec^2]','FontSize',12)
title('Accleration Limits','FontSize',15)
grid on;

figure;
plot(time,eyL1,'LineWidth',2)
xlabel('time [sec]','FontSize',12)
ylabel('Tracking error [m]','FontSize',12)
title('Tracking Error During Maneuver','FontSize',15)
grid

figure
plot(time,vel1,'LineWidth',3);
xlabel('Time [s]','FontSize',12);
ylabel('Velocity [m/s]','FontSize',12);
title('Velocity Change During Maneuver','FontSize',15)
grid on

figure;
plot(xVec,yVec,'LineWidth',3);
hold on
plot(Xcurve1,Ycurve1,'r','LineWidth',3,'LineStyle','-.')
xlabel('X-position [m]','FontSize',12);
ylabel('Y-position [m]','FontSize',12);
title('Trajectory Comparison','FontSize',15)
legend('Original Traj','Simulation','FontSize',13)
grid on

figure;
plot(time,Ycurve1,'LineWidth',3);
xlabel('Time [s]','FontSize',12);
ylabel('Y-Position [m]','FontSize',12);
title('Y-Positions vs Time','FontSize',15);
grid on

figure;
plot(time,acc_lat1,'LineWidth',3);
xlabel('Time [s]','FontSize',12);
ylabel('Acc-Lateral','FontSize',12);
title('Acc-Lat vs Time','FontSize',15);
grid on