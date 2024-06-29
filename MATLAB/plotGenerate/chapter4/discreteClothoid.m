function [CircleData,LineData,EP,theta_end,xVec,yVec] = discreteClothoid(S,n,kA,kB,PS,thetaS,plotOn)
% Format of CircleData
% Cx: center x
% Cy: center y
% R: radius
% phi_s: start angle
% phi_e: end angle
% l: arc length
% Format of LineData

% ===================
lineColor = 'k';
k = kB-kA;
h = (kB-kA)/n;
lane = 4;
kVec = [kA+h:h:kA+n*h];
if kB == 0
    kVec(end) = 0;
end
CircleData = [];
LineData = [];
% Record path parameters
CXvec = [];
CYvec = [];
Radiusvec = [];
theta_svec = [];
theta_evec = [];
% First arc
if kA == 0
    w0 = 0;
else
    w0 = S*kA/2/n;
    R0 = 1/kA;
end

for j = 1:n-1
    wVec(j) = S*kVec(j)/n;
    Rvec(j) = 1/kVec(j);
end
wVec(n) = S*kVec(end)/2/n;
Rvec(n) = 1/kVec(end);
xVec = []; 
yVec = []; 

% Simulate clothoid trajectory for comparison
if plotOn
    options = simset('SrcWorkspace','current');
    sim('clothoidSimulation1.slx',S,options);
    plot(Xval,Yval,'b');
    hold all;
    grid on;
end

% First arc
if kA == 0
    L0 = S/2/n;
    X1 = PS(1)+L0*cos(thetaS);
    Y1 = PS(2)+L0*sin(thetaS);
    AX = [PS(1) X1];
    AY = [PS(2) Y1];
    if plotOn
        plot(AX,AY,lineColor)
    end
    xVec = [xVec AX]; 
    yVec = [yVec AY]; 
%     plot([0 X1],[Y1+lane Y1+lane])
    R0 = Inf;
    phi = [thetaS thetaS];
    CX = X1;
    CY = Y1;
    LineData = [LineData [PS; CX; CY; L0] ];
else
    CX = PS(1)-R0*sin(thetaS);
    CY = PS(2)+R0*cos(thetaS);
    if kA > 0
        phi = unique([thetaS:.0001:w0+thetaS w0+thetaS]);
    else
        phi = fliplr(unique([w0+thetaS:.0001:thetaS thetaS]) );
    end
    AX = CX+R0*sin(phi);
    AY = CY-R0*cos(phi);
%     BX = CX+(R0-lane)*sin(phi);
%     BY = CY-(R0-lane)*cos(phi);
    if plotOn
        plot(AX,AY,lineColor)
    end
    xVec = [xVec AX]; 
    yVec = [yVec AY]; 
%     plot(BX,BY)
%     plot([CX AX(1)],[CY AY(1)],'k--')
%     plot([CX AX(end)],[CY AY(end)],'k--')
    CircleData = [CircleData [CX; CY; R0; phi(1); phi(end); S/2/n] ]; 
end


% Second arc
if wVec(1) >=0
    phi = unique([w0+thetaS:.0001:w0+thetaS+wVec(1) thetaS+w0+wVec(1)]);
else
    phi = fliplr(unique([thetaS+w0+wVec(1):.0001:w0+thetaS w0+thetaS]) );
end

if wVec(1) == 0
    CX = AX(end);
    CY = AY(end);
    AX = CX+[0:.001:S/n]*cos(w0+thetaS);
    AY = CY+[0:.001:S/n]*sin(w0+thetaS);
    LineData = [LineData [CX; CY; AX(end); AY(end); S/n] ];
%     BX = AX;
%     BY = AY+lane;
else
    if w0 == 0
        CX = AX(end)-Rvec(1)*sin(thetaS);
        CY = AY(end)+Rvec(1)*cos(thetaS);
    else
        CX = CX+(R0-Rvec(1))*sin(w0+thetaS);
        CY = CY-(R0-Rvec(1))*cos(w0+thetaS);
    end
    AX = CX+Rvec(1)*sin(phi);
    AY = CY-Rvec(1)*cos(phi);
%     BX = CX+(Rvec(1)-lane)*sin(phi);
%     BY = CY-(Rvec(1)-lane)*cos(phi);
    CXold = CX;
    CYold = CY;
    CircleData = [CircleData [CX; CY; Rvec(1); phi(1); phi(end); S/n] ]; 
end

if plotOn
    plot(AX,AY,lineColor)
end
    xVec = [xVec AX]; 
    yVec = [yVec AY]; 
% plot(BX,BY)
% plot([CX AX(1)],[CY AY(1)],'k--')
% plot([CX AX(end)],[CY AY(end)],'k--')

for j = 2:n
    if wVec(j) >= 0
        phi = unique([thetaS+w0+sum(wVec(1:j-1) ):.0001:thetaS+w0+sum(wVec(1:j) ) thetaS+w0+sum(wVec(1:j) )] );
    else
        phi = fliplr(unique([thetaS+w0+sum(wVec(1:j) ):.0001:thetaS+w0+sum(wVec(1:j-1) ) thetaS+w0+sum(wVec(1:j-1) )] ) );
    end
    
    if wVec(j-1) == 0
        if j == 2
            CX = CXold-Rvec(j)*sin(w0+thetaS);
            CY = CYold+Rvec(j)*cos(w0+thetaS);
        else
            CX = CXold-Rvec(j)*sin(thetaS+w0+sum(wVec(1:j-2) ) ); % slope of previous straight line
            CY = CYold+Rvec(j)*cos(thetaS+w0+sum(wVec(1:j-2) ) );
        end
    else
        if wVec(j) == 0
            if wVec(j-1) < 0
                CX = AX(1);
                CY = AY(1);
            else
                CX = AX(end);
                CY = AY(end);
            end
        else
            CX = CXold+(Rvec(j-1)-Rvec(j))*sin(thetaS+w0+sum(wVec(1:j-1) ) );
            CY = CYold-(Rvec(j-1)-Rvec(j))*cos(thetaS+w0+sum(wVec(1:j-1) ) );
        end
    end
    
    if wVec(j) == 0
        CX = AX(end);
        CY = AY(end); 
        if j < n
            AX = CX+unique([0:.001:S/n/2 S/n])*cos(thetaS+w0+sum(wVec(1:j-1)));
            AY = CY+unique([0:.001:S/n/2 S/n])*sin(thetaS+w0+sum(wVec(1:j-1)));
        else
            CX = AX(end); 
            CY = AY(end); 
            AX = CX+unique([0:.001:S/n/2 S/n/2])*cos(thetaS+w0+sum(wVec(1:j-1)));
            AY = CY+unique([0:.001:S/n/2 S/n/2])*sin(thetaS+w0+sum(wVec(1:j-1)));
        end
%         BX = AX-lane*sin(thetaS+w0+sum(wVec(1:j-1) ) );
%         BY = AY+lane*cos(thetaS+w0+sum(wVec(1:j-1) ) );
        CXold = AX(end);
        CYold = AY(end);   
        if j < n
            LineData = [LineData [CX; CY; AX(end); AY(end); S/n] ];
        else
            LineData = [LineData [CX; CY; AX(end); AY(end); S/2/n] ];


        end
    else
        AX = CX+Rvec(j)*sin(phi);
        AY = CY-Rvec(j)*cos(phi);  
        BX = CX+(Rvec(j)-lane)*sin(phi);
        BY = CY-(Rvec(j)-lane)*cos(phi);   
        CXold = CX;
        CYold = CY;
        if j < n
            CircleData = [CircleData [CX; CY; Rvec(j); phi(1); phi(end); S/n] ]; 
        else
            CircleData = [CircleData [CX; CY; Rvec(j); phi(1); phi(end); S/2/n] ]; 
        end
    end
    if plotOn
        plot(AX,AY,lineColor)
    end
    xVec = [xVec AX]; 
    yVec = [yVec AY]; 
%     plot(BX,BY)
%     plot([CX AX(1)],[CY AY(1)],'k--')
%     plot([CX AX(end)],[CY AY(end)],'k--')
end

theta_end = phi(end);
SP = [AX(1); AY(1)];
EP = [AX(end); AY(end)];
% 
% plot([SP(1) EP(1)],[SP(2) EP(2)])
% Svec = SP';
% Evec = EP';
% LineData = [LineData [SP'; EP']];
