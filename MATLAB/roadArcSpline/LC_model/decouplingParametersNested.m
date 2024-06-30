clc

v = v0;
dx0 = v;
y0 = 0;
dy0 = 0;
dpsi0 = 0;
kappaf = 0;

%% Vehicle Parameters

a = 1.421; %CG'den one
b = 1.029; %CG'den arkada
m = 1480; % Weight
g = 9.81; 
Izz = 1950; %inertia
cf = -1.5e5*1; %front wheel coefficient
cr = -1.5e5*1; %rear wheel coefficient

D = -muVal*m*g;
C = 1.65;
B = 8.22;
E = -10;

% l_S = 10;

%% Controller Parameters
% taui = .2;
% 
% Kp1 = 0.1;
% Kd1 = 0.04;
% Kp2 = 0.3;
% % Test controller for Ls = 12
% Kp1 = .1;
% Kd1 = .05;
% Kp2 = .3;
% Kd2 = 0;
% % Test controller for Ls = 2 and steering 0.01
% l_S = 2;
% Kp1 = .3;
% Kd1 = 0.0;
% Kp2 = .3;
% Kd2 = .0;
% % Test controller for Ls = 4 and steering 0.05 and dyL
% l_S = 4;
% Kp1 = .01;
% Kd1 = 0.01;
% Kp2 = .005;
% Kd2 = .01;
% % Test controller for Ls = 4 and steering 0.05 and ddeltaPsi
% l_S = 2;
% Kp1 = 1;
% Kd1 = 0.2;
% Kp2 = .1;
% Kd2 = .1;

%% Controller Parameters
%TODO
% KP1 = 20;
% KI1 = 10;
% KP2 = 30;
% KI2 = 0.01; 
% KI3 = 0.01; 
% Kd = 0.05;
% tau = 0.01;

% KP1 = 20;
% KI1 = 10;
% KP2 = 30;
% KI2 = 0.01; 
% KI3 = 0.01; 
% Kd = 0.05;
% tau = 0.01;

% Working set ATAKAN
KP1 = 20;
KI1 = 15;
KP2 = 30;
KI2 = 0.03; 
KI3 = 0.03; 
Kd = 0.05;
tau = 0.01;

% l_S = 1.25;
l_S = 4.75; % look ahead distance, when we increase it creates problems
%% System Dynamics

v_LC = v0; % Lateral system dynamics with this constant velocity

lf = a;
lr = b;
J = Izz;
a11 = -(cf + cr)/m/v_LC;
a12 = -1 - (cf*lf - cr*lr)/m/v_LC^2;
a21 = -(cf*lf-cr*lr)/J;
a22 = - (cf*lf^2+cr*lr^2)/J/v_LC;
b1 = cf/m/v_LC;
b2 = cf*lf/J;

Asys = [a11 a12 0 0;
        a21 a22 0 0;
        0 1 0 0;
        v_LC l_S v_LC 0];

B1 = [b1; b2; 0; 0];
B2 = [0; 0; -v_LC; 0];
Bsys = [B1 B2];
Csys = [0 1 0 0;
        0 0 0 1];
Dsys = [0 0; 0 0];

G1ss = ss(Asys,B1,[0 1 0 0],0);
G1 = tf(G1ss);

K = freqresp(G1,0);
