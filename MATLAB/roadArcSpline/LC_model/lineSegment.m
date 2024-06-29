function LineData = lineSegment(SP,thetaS,L,plotOn)
EP = SP + L*[cos(thetaS);sin(thetaS)];
LineData = [SP;EP;L];
if plotOn
    plot([SP(1) EP(1)],[SP(2) EP(2)],'g','LineWidth',3)
end