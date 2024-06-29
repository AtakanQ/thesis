% In this code we can see the curvature value k1 depending on the S value.

syms lambda gamma k1 a mu g v0 S s positive

aVal = 3;
v0Val = 12;
muVal = .82;
gVal = 9.81;
gammaVal = 1;

kMax = sqrt(mu^2*g^2-a^2)/(v0^2+2*a*s);

k1 = subs(kMax,s,lambda*gamma*S/2);
k2 = subs(kMax,s,S-(1-lambda)*gamma*S/2);
lambdaSol = solve(  k2 == lambda/(1-lambda)*k1,lambda);
subs(lambdaSol,[a mu g v0],[aVal muVal gVal v0Val]);
k1Sol = subs(k1,lambda,lambdaSol(2) );

diff(lambdaSol(2),lambda)
diff(lambdaSol(2),S)


lambdaSol =  -(S*aVal - (S^2*aVal^2*gammaVal^2 - 2*S^2*aVal^2*gammaVal ...
    + S^2*aVal^2 + 2*S*aVal*v0Val^2 + v0Val^4)^(1/2) ...
    + v0Val^2 - S*aVal*gammaVal)/(2*S*aVal*gammaVal);
SvalVec = [20:5:100];
lambdaSolVec = subs(lambdaSol,S,SvalVec);
figure
subplot(2,1,1)
plot(SvalVec,lambdaSolVec,'LineWidth',3)
xlabel('S [arc length]');
ylabel('Lambda [ratio of S1&S2]');
title([' a = ',num2str(aVal),' -------',' v0 = ',num2str(v0Val)],'FontSize',20);
grid on
k1SolVec = subs((gVal^2*muVal^2 - aVal^2)^(1/2)./(v0Val^2 + S*aVal.*gammaVal.*lambdaSol),S,SvalVec);
subplot(2,1,2)
plot(SvalVec,k1SolVec,'LineWidth',3)
xlabel('S [arc length]','FontSize',15);
ylabel('k1 [curvature]','FontSize',15);
grid on
