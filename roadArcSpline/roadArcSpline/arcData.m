function ArcData = arcSegment(CP,R,thetaS,L)
% CP: center
% R: radius
% L: length
if L > 0
    if R > 0
        phi = unique([thetaS:.0001:thetaS+L/R thetaS+L/R]);
    else
        phi = unique([thetaS+L/R:.0001:thetaS thetaS]);
    end
    AX = CP(1)+R*sin(phi);
    AY = CP(2)-R*cos(phi);
    plot(AX,AY,'g')
    ArcData = [CP; R; thetaS; thetaS+L/R; L];
else
    ArcData = [];
end