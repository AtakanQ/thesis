
aVal = -1;
v0Val = 20;
muVal = .5;
gVal = 9.81;

k1Val = .03;
lambdaVal = .3;
gammaVal = 1;
deltaY = 3.7;
% Compute change of c along parameters
[alpha,Sval] = computeAlphaS(k1Val,lambdaVal,gammaVal,deltaY);

kk = 1;
lambdaSolVec = [];
k1SolVec = [];
% First iteration: Upper bound on gamma
while 1
        % Compute acceleration along arc length
        kMaxVec = sqrt(muVal^2*gVal^2-aVal^2)./(v0^2+2*a*Svec).^2;

        lambdaSol = -(Sval*aVal - (- Sval^2*aVal^2*gammaVal^2 + 2*Sval^2*aVal^2*gammaVal ...
            + Sval^2*aVal^2 + 2*Sval*aVal*v0Val^2 + v0Val^4)^(1/2) + v0Val^2 ... 
            - Sval*aVal*gammaVal)/(2*Sval*aVal*gammaVal);
        lambdaSolVec(kk) = lambdaSol;
        k1Sol = (gVal^2*muVal^2 - aVal^2)^(1/2)/(v0Val^2 + Sval*aVal*gammaVal*lambdaSol);
        k1SolVec(kk) = double(k1Sol);
        [alpha,SSol] = computeAlphaS(k1Sol,lambdaSol,gammaVal,deltaY);
        if abs(SSol-Sval) < 1E-6
            break;
        end
        Sval = SSol;
end
SSolUp = SSol;

kk = 1;
lambdaSolVec = [];
k1SolVec = [];
gammaVal = .5;
% First iteration: Lower bound of gamma
while 1
        % Compute acceleration along arc length
        kMaxVec = sqrt(muVal^2*gVal^2-aVal^2)./(v0^2+2*a*Svec).^2;

        lambdaSol = -(Sval*aVal - (- Sval^2*aVal^2*gammaVal^2 + 2*Sval^2*aVal^2*gammaVal ...
            + Sval^2*aVal^2 + 2*Sval*aVal*v0Val^2 + v0Val^4)^(1/2) + v0Val^2 ... 
            - Sval*aVal*gammaVal)/(2*Sval*aVal*gammaVal);
        lambdaSolVec(kk) = lambdaSol;
        k1Sol = (gVal^2*muVal^2 - aVal^2)^(1/2)/(v0Val^2 + Sval*aVal*gammaVal*lambdaSol);
        k1SolVec(kk) = double(k1Sol);
        [alpha,SSol] = computeAlphaS(k1Sol,lambdaSol,gammaVal,deltaY);
        if abs(SSol-Sval) < 1E-6
            break;
        end
        Sval = SSol;
end
SSolLow = SSol;
    

% Bisection
gammaUp = 1;
gammaLow = .5;
Sold = 1000;
gammaVec = .5:.1:1;
for jj = 1:length(gammaVec)
    gammaVal = gammaVec(jj);
%     gammaVal = .5*(gammaUp+gammaLow);
    lambdaSolVec = [];
    kk = 1;
    while 1
        % Compute acceleration along arc length
        

        lambdaSol =  -(Sval*aVal - (Sval^2*aVal^2*gammaVal^2 - 2*Sval^2*aVal^2*gammaVal ...
            + Sval^2*aVal^2 + 2*Sval*aVal*v0Val^2 + v0Val^4)^(1/2) ...
            + v0Val^2 - Sval*aVal*gammaVal)/(2*Sval*aVal*gammaVal);
        lambdaSolVec(kk) = lambdaSol;
        k1Sol = (gVal^2*muVal^2 - aVal^2)^(1/2)/(v0Val^2 + Sval*aVal*gammaVal*lambdaSol);
        k1SolVec(kk) = k1Sol;
        k2Sol = (gVal^2*muVal^2 - aVal^2)^(1/2)/(v0Val^2 + 2*aVal*(Sval-(1-lambdaSol)*gammaVal*Sval/2 ));
%         (2*(g^2*mu^2 - a^2)^(1/2))/((S^2*a^2*gamma^2 - 2*S^2*a^2*gamma + S^2*a^2 + 2*S*a*v0^2 + v0^4)^(1/2) - S*a + v0^2 + S*a*gamma)

        [alpha,SSol] = computeAlphaS(k1Sol,lambdaSol,gammaVal,deltaY);
        SSolVec(kk) = SSol;
        kk = kk+1;
        if abs(SSol-Sval) < 1E-6
            break;
        end
        Sval = SSol;
    end
    SIt(jj) = SSol;
    lambdaIt(jj) = lambdaSol;
    k1It(jj) = k1Sol;
    if SSol < Sold
        SOpt = SSol;
        gammaOpt = gammaVal;
        lambdaOpt = lambdaSol;
        k1Opt = k1Sol;
    end
    if gammaUp-gammaLow < 1E-6
        break;
    end
end
% Evaluation of result
Sval = SOpt;
lambdaVal = lambdaOpt; 
gammaVal = gammaOpt;
k1Val = k1Opt; 

n = 5;
plotOn = 1;
[CircleData,LineData] = clothoidLCComputation(Sval,k1Val,lambdaVal,gammaVal,1,n,plotOn);
[arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData);
figure
subplot(2,1,1)
plot(xVec,yVec)

S1 = gammaVal*lambdaVal*Sval;
S2 = gammaVal*(1-lambdaVal)*Sval;
SL = (1-gammaVal)*Sval;

n1 = 20;
nL = 10;
n2 = 20;

h1 = gammaVal*lambdaVal*Sval/n1;
hL = (1-gammaVal)*Sval/nL;
h2 = gammaVal*(1-lambdaVal)*Sval/n2;


S1vec = ([0:h1:n1*h1]);
if gammaVal < 1
    SLvec = ([0:hL:nL*hL]);
else
    SLvec = zeros(1,nL+1);
end
S2vec = ([0:h2:n2*h2]);

% arc points under study
Svec = ([S1vec SLvec+S1vec(end) S2vec+S1vec(end)+SLvec(end)]);
% % related velocities
% 
vVec = v0Val;
kVec = 0;
k2Val = -lambdaVal*k1Val/(1-lambdaVal);
for jj = 2:length(Svec)
   vVec(jj) = vVec(jj-1) + 2*(-vVec(jj-1)+sqrt(vVec(jj-1)^2+aVal*(Svec(jj)-Svec(jj-1))/2) );
   if Svec(jj) <= S1/2
       kVec(jj) = k1Val*2*Svec(jj)/S1;
   elseif Svec(jj) <= S1
       kVec(jj) = k1Val*2*(S1-Svec(jj))/S1;
   elseif Svec(jj) <= S1+SL
       kVec(jj) = 0;
   elseif Svec(jj) <= S1+SL+S2/2
       kVec(jj) = k2Val*2*(Svec(jj)-S1-SL)/S2;
   else
       kVec(jj) = k2Val*2*(S1+S2+SL-Svec(jj))/S2;
   end
end
kMaxVec = sqrt(muVal^2*gVal^2-aVal^2)./(v0Val^2+2*aVal*Svec);

subplot(2,1,2)
plot(Svec,kVec)
hold all;
plot(Svec,kMaxVec,'k--')
plot(Svec,-kMaxVec,'k--')