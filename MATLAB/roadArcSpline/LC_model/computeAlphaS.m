function [alpha,S,kk] = computeAlphaS(k1,lambda,gamma,deltaY)

syms alphaSym z;

Dintegrand =  @(x,alphaSym) cos(2*alphaSym*(-x.^2+x) );
alphaVec = -pi/4:.01:pi/4;
DalphaVec = [];
DalphaSinVec = [];
for jj = 1:length(alphaVec)
    alphaSym = alphaVec(jj);
    DalphaVec(jj) = 2*integral(@(x)Dintegrand(x,alphaSym),0,.5);
    DalphaSinVec(jj) = DalphaVec(jj)*sin(alphaSym/2);
end
P = polyfit(alphaVec,DalphaVec,3);
DalphaPoly = @(alph)(P(4) + P(3)*alph + P(2)*alph.^2 + P(1)*alph.^3);
diffDalphaPoly = @(alph)(P(3)+2*P(2)*alph+3*P(1)*alph.^2);

epsilon = 1E-8;
funck1 =  @(alph)(2*alph.*(gamma*DalphaPoly(alph).*sin(alph/2)+(1-gamma)*sin(alph))-deltaY*k1*gamma*lambda);
dfunck1 = @(alph)((2*alph*(gamma*sin(alph/2)*diffDalphaPoly(alph) - cos(alph)*(gamma - 1) + (gamma*cos(alph/2)*DalphaPoly(alph))/2)) - (2*(sin(alph)*(gamma - 1) - gamma*sin(alph/2)*DalphaPoly(alph))));

alphaVec = [0:.01:pi/4 pi/4];
funcVal = funck1(alphaVec);
% figure;
% hold all;
% plot(alphaVec,funcVal);
x = pi/4;
kk = 0;
while abs(funck1(x)) > epsilon
    kk = kk+1;
   xOld = x;
   x = -funck1(x)/dfunck1(x)+x; 
%    plot([xOld x],[funck1(xOld) 0]);
end
% grid on;
alpha = x;

S = alpha*2/k1/lambda/gamma;