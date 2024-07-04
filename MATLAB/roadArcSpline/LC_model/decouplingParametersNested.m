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
m = 1100; % Weight
g = 9.81; 
Izz = 1950; %inertia
cf = -1.5e5*1; %front wheel coefficient
cr = -1.5e5*1; %rear wheel coefficient

D = -muVal*m*g;
C = 1.65;
B = 8.22;
E = -10;

% l_S = 10;

% Working set ATAKAN
KP1 = 25;
% KP1 = 5;
KI1 = 15;
KP2 = 40;
% KP2 = 10;
KI2 = 0.03; 
KI3 = 0.03; 
Kd = 0.05;
% Kd = 0.01;
tau = 0.01;

% l_S = 1.25;
l_S = 1.25; % look ahead distance, when we increase it creates problems
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
