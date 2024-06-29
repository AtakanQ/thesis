function [CircleData,LineData,DataX,DataY] = discreteClothoid_omer(S,n,kA,kB,PS,thetaS,plotOn)

lineColor = 'k';
k = kB-kA; % curvature difference
h = (kB-kA)/n; % curvature step size


kVec = [kA+h:h:kA+n*h]; % curvature vector

% If final curvature is zero then put kVEc final to zero
if kB == 0
    kVec(end) = 0;
end

CircleData = []; % Defines Center,Radius,Angle etc
LineData = [];
DataX = [];
DataY = [];

% First arc
if kA == 0 % if first curvature is zero
    w0 = 0;
else
    w0 = S*kA/2/n;
    R0 = 1/kA;
end

% Here obtain the angle and radius vector

for j = 1:n-1
    wVec(j) = S*kVec(j)/n;
    Rvec(j) = 1/kVec(j);
end

wVec(n) = S*kVec(end)/2/n;
Rvec(n) = 1/kVec(end);


% Simulate clothoid trajectory for comparison
if plotOn
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
        plot(AX,AY,lineColor,'LineWidth',3)
    end
    
    R0 = Inf;
    phi = [thetaS thetaS];
    CX = X1;
    CY = Y1;
    LineData = [LineData [PS; CX; CY; L0] ];
else
    CX = PS(1)-R0*sin(thetaS);
    CY = PS(2)+R0*cos(thetaS);
    
    if kA > 0
        phi = unique([thetaS:.01:w0+thetaS w0+thetaS]);
    else
        phi = fliplr(unique([thetaS:-.01:w0+thetaS w0+thetaS]));
    end
    AX = CX+R0*sin(phi);
    AY = CY-R0*cos(phi);
    
    if plotOn
        plot(AX,AY,lineColor,'LineWidth',3)
    end
    
    CircleData = [CircleData [CX; CY; R0; phi(1); phi(end); S/2/n] ];
end

DataX = [DataX AX(1:end)];
DataY = [DataY AY(1:end)];

% Second arc
if wVec(1) >=0
    phi = unique([w0+thetaS:.01:w0+thetaS+wVec(1) thetaS+w0+wVec(1)]);
else
    phi = fliplr(unique([thetaS+w0+wVec(1):.01:w0+thetaS w0+thetaS]) );
end

if wVec(1) == 0
    CX = AX(end);
    CY = AY(end);
    AX = CX+[0:.01:S/n]*cos(w0+thetaS);
    AY = CY+[0:.01:S/n]*sin(w0+thetaS);
    LineData = [LineData [CX; CY; AX(end); AY(end); S/n] ];
    
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
    
    CXold = CX;
    CYold = CY;
    CircleData = [CircleData [CX; CY; Rvec(1); phi(1); phi(end); S/n] ];
end

if plotOn
    plot(AX,AY,lineColor,'LineWidth',3)
end

DataX = [DataX AX(2:end)];
DataY = [DataY AY(2:end)];

for j = 2:n
    if wVec(j) >= 0
        phi = unique([thetaS+w0+sum(wVec(1:j-1)):.01:thetaS+w0+sum(wVec(1:j) ) thetaS+w0+sum(wVec(1:j) )] );
    else
        phi = fliplr(unique([thetaS+w0+sum(wVec(1:j)):.01:thetaS+w0+sum(wVec(1:j-1) ) thetaS+w0+sum(wVec(1:j-1) )] ) );
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
            
            CX = AX(end);
            CY = AY(end);
            
        else
            CX = CXold+(Rvec(j-1)-Rvec(j))*sin(thetaS+w0+sum(wVec(1:j-1) ) );
            CY = CYold-(Rvec(j-1)-Rvec(j))*cos(thetaS+w0+sum(wVec(1:j-1) ) );
        end
    end
    
    if wVec(j) == 0
        if j < n
            AX = CX+[0:.01:S/n]*cos(thetaS+w0+sum(wVec(1:j-1)));
            AY = CY+[0:.01:S/n]*sin(thetaS+w0+sum(wVec(1:j-1)));
        else
            AX = CX+[0:.01:S/n/2]*cos(thetaS+w0+sum(wVec(1:j-1)));
            AY = CY+[0:.01:S/n/2]*sin(thetaS+w0+sum(wVec(1:j-1)));
        end
        
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
        CXold = CX;
        CYold = CY;
        if j < n
            CircleData = [CircleData [CX; CY; Rvec(j); phi(1); phi(end); S/n] ];
        else
            CircleData = [CircleData [CX; CY; Rvec(j); phi(1); phi(end); S/2/n] ];
        end
    end
    
    if plotOn
        plot(AX,AY,lineColor,'LineWidth',3)
    end
    
    
    DataX = [DataX AX(2:end)];
    DataY = [DataY AY(2:end)];
    
    % if plotOn
    %     plot(DataX,DataY,lineColor,'LineWidth',3,'LineStyle','-.')
    % end

    
end


