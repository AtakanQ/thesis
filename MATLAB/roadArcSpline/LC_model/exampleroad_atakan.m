%% Trial for LC maneuver with traj data
clear
% Load here LC_startf,S_numf,deltaYf,k1f,lambdaf
addpath("..\")
load LCdata_trial_13ms


fileName = "case1";
% fileName = "case2";
% fileName = "case3";

load("mainFitArc"+fileName+".mat")
% load LCdata_trial_20ms

%% Maneuver Data Entrance

% v_ent = LCdata.v_ent; % 
v0_LC = LCdata.v0;
% S_numf = LCdata.S_numf;
% k1f = LCdata.k1f;
% lambdaf = LCdata.lambdaf;
% LC_startf = LCdata.LC_startf;
% 
% acc = (v_ent^2-v0_LC^2)/(2*LC_startf); % deleceration value
acc = 0;

% End = LCdata.End;
% Start2 = LCdata.Start2;
% deltaYf = LCdata.deltaYf;
% 
% deltaY2 = -deltaYf;
% k1_mir = lambdaf/(1-lambdaf)*k1f*sign(deltaY2);
% lambda_mir = 1-lambdaf;

n = 5; % discretization points
%% Trajectory Re-generation

Sall = 0;
SLC = 0;
CircleData = [];
LineData = [];
plotOn = 1;
pointCount = 1;
curvVec = [clothoidArray(1).init_curv];
curvArc = [0];
% ============================
% Straight
% ============================
% figure;
% hold all;
% S = LC_startf;
% SP = [0;0];
% thetaS = 0;
% Sall = Sall+S;
% [LineDataNew] = lineSegment(SP,thetaS,S,plotOn);
% LineData = [LineData [LineDataNew; pointCount:pointCount+size(LineDataNew,2)-1] ];
% pointCount = pointCount+size(LineDataNew,2);
% PS = LineData(3:4,end); % should be LC_startf
% thetaS = 0;
% curvArc = [0:1:Sall];
% curvVec = zeros(size(curvArc));
PS = [0; 0];
thetaS = 0;
for i = 1:numel(clothoidArray)
    S = clothoidArray(i).curv_length;
    Sall = Sall+S;
    kA = clothoidArray(i).init_curv;
    kB = clothoidArray(i).final_curv;
    [CircleDataNew,LineDataNew,~,~] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn);
    LineData = [LineData [LineDataNew;pointCount:pointCount+size(LineDataNew,2)-1] ];
    pointCount = pointCount+size(LineDataNew,2);
    CircleData = [CircleData [CircleDataNew;pointCount:pointCount+size(CircleDataNew,2)-1] ];
    pointCount = pointCount+size(CircleDataNew,2);
    Cend = CircleData(1:2,end);
    Rend = CircleData(3,end);
    thetaS = CircleData(5,end);
    PS = Cend + Rend*[sin(thetaS); -cos(thetaS)];


    curvArc = [curvArc (curvArc(i)+S)];
    curvVec = [curvVec kB];
end

[arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData); % no need for arcVec

% xVec = arcSpline.allX; orijinden başlayacak şekilde kullanmalı
% yVec = arcSpline.allY; orijinden başlayacak şekilde kullanmalı

figure;
plot(xVec,yVec,'LineWidth',2);
axis equal
xlabel('X-position [m]','FontSize',13)
ylabel('Y-position [m]','FontSize',13)
set(gca,'FontSize',13)

muVal = .82;
g = 9.81;


%% Simulation Time

% Compute velocity profile
v0 = 36; % initial velocity
tSim = Sall/v0; % simulation time 
% tSim = 1.11;
acc_sim = 0; % constant velocity 
S01 = 0; % acc = 0 ise sifir olabilir
v1 = v0; % maneuver velocity
acc1 = 0; % gereksiz sifir olsa iyi

% decouplingParameters_yL
decouplingParametersNested
LineData =[
         0;
         0;
        0;
         0;
        0;
     0];
% sim('laneKeepingArcSplinesNestedRoad_2013a.slx',tSim)
sim('laneKeepingArcSplinesNested_atakan.slx',tSim)


%% Plots
close all
vmaxVec = sqrt(sqrt(muVal^2*g^2)./abs(curvVec));
figure;

subplot(2,1,1)
plot(curvArc,curvVec,'LineWidth',2)
xlabel('arc-length [m]','FontSize',13)
ylabel('road curvature $k_{\mathcal{T}}$','FontSize',13,'Interpreter','latex')
grid on
title("Curvature of Trajectory","FontSize",13)
subplot(2,1,2)
plot(curvArc,vmaxVec,'LineWidth',2)
xlabel('arc-length [m]','FontSize',13)
ylabel('velocities [m/sec]','FontSize',13)
ylim([0 200])
title("Maximum Velocity over Trajectory","FontSize",13)
grid
hold all;
plot(arclength1,vel1,'LineWidth',2)
lll = legend('$v_{\mathrm{max}}$','$\mathcal{V}(s)$','Location','best');
set(lll,'Interpreter','latex')
grid on

% set(gca,'Ylim',[20 50])

figure;
plot([0 arclength1(end)],[muVal*g muVal*g],'k--','LineWidth',2)
hold all;
%plot(arclength1, abs(acc_long1),'LineWidth',2)
plot(arclength1, sqrt(acc_long1.^2+acc_lat1.^2 ),'LineWidth',2)
lll = legend('$\mu\cdot g$','$\sqrt{a_{long}\sp2 + a_{lat}\sp2}$','Location','best');
set(lll,'FontSize',13,'Interpreter','latex')
xlabel('X-position [m]','FontSize',13)
ylabel('accelerations [m/sec^2]','FontSize',13)
title('Acceleration Limits','FontSize',13)
grid on;

figure;
plot(time(1:(end-1)),eyL1(1:(end-1)),'LineWidth',2)
xlabel('time [sec]','FontSize',13)
ylabel('Tracking error [m]','FontSize',13)
title('Tracking Error During Maneuver','FontSize',13)
grid on

figure;
plot(xVec,yVec,'LineWidth',3);
hold on
plot(Xcurve1,Ycurve1,'r','LineWidth',3,'LineStyle','-.')
xlabel('X-position [m]','FontSize',13);
ylabel('Y-position [m]','FontSize',13);
title('Trajectory Comparison','FontSize',13)
legend('Original Traj','Simulation','FontSize',13,'Location','Best')
grid on
axis equal
