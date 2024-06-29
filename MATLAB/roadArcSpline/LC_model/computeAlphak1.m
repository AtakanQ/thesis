function [alpha,k1] = computeAlphak1(S,lambda,gamma,deltaY)

syms alphaSym z;

Dintegrand =  @(x,alphaSym) cos(2*alphaSym*(-x.^2+x) );
alphaVec = 0:.01:pi/4;
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
funcS =  @(alph)((gamma*DalphaPoly(alph).*sin(alph/2)+(1-gamma)*sin(alph))-deltaY/S);
dfuncS = @(alph)(gamma*sin(alph/2).*diffDalphaPoly(alph) - cos(alph)*(gamma - 1) + (gamma*cos(alph/2).*DalphaPoly(alph))/2);

alphaVec = [0:.01:pi/4 pi/4];
funcVal = funcS(alphaVec);
figure;
hold all;
plot(alphaVec,funcVal);
x = 0;
while abs(funcS(x)) > epsilon
    xOld = x;
    x = -funcS(x)/dfuncS(x)+x; 
    plot([xOld x],[funcS(xOld) 0]);
end
grid on;
alpha = x;

k1 = 2*alpha/S/lambda/gamma;