function [CircleData,LineData,curvArc,curvVec,EP,theta_end,xVec,yVec] = computeBiElementaryPath(S,lambda,gamma,n,k1,PS,thetaS,plotOn)
    xVec = []; 
    yVec = []; 
    Sall = 0;
    curvVec = [];
    curvArc = [];
    k2 = -lambda/(1-lambda)*k1; 
    SE1 = lambda*gamma*S; 
    SL = (1-gamma)*S; 
    SE2 = (1-lambda)*gamma*S; 
    [CircleData,LineData,EP,theta_end,xVec,yVec] = discreteClothoid(SE1/2,n,0,k1,PS,thetaS,plotOn);
    Sall = Sall+SE1/2;
    curvArc = [curvArc unique([Sall-SE1/2:1:Sall Sall])];
    curvVec = [curvVec k1/SE1*2*(unique([Sall-SE1/2:1:Sall Sall])-(Sall-SE1/2)) ];
    [CircleDataNew,LineDataNew,EP,theta_end,xVec2,yVec2] = discreteClothoid(SE1/2,n,k1,0,EP,theta_end,plotOn);
    xVec = [xVec xVec2]; 
    yVec = [yVec yVec2]; 
    CircleData = [CircleData CircleDataNew]; 
    LineData = [LineData LineDataNew]; 
    Sall = Sall+SE1/2;
    curvArc = [curvArc unique([Sall-SE1/2:1:Sall Sall])];
    curvVec = [curvVec k1/SE1*2*(Sall-unique([Sall-SE1/2:1:Sall Sall])) ];
    [LineDataNew,EP,xVec2,yVec2] = lineSegment(EP,theta_end,SL,plotOn); 
    xVec = [xVec xVec2]; 
    yVec = [yVec yVec2];  
    LineData = [LineData LineDataNew];
    Sall = Sall+SL;
    curvArc = [curvArc unique([Sall-SL:1:Sall Sall])];
    curvVec = [curvVec zeros(size(unique([Sall-SL:1:Sall Sall])))];
    [CircleDataNew,LineDataNew,EP,theta_end,xVec2,yVec2] = discreteClothoid(SE2/2,n,0,k2,EP,theta_end,plotOn);
    xVec = [xVec xVec2]; 
    yVec = [yVec yVec2]; 
    CircleData = [CircleData CircleDataNew]; 
    LineData = [LineData LineDataNew]; 
    Sall = Sall+SE2/2;
    curvArc = [curvArc unique([Sall-SE2/2:1:Sall Sall])];
    curvVec = [curvVec k2/SE2*2*(unique([Sall-SE2/2:1:Sall Sall])-(Sall-SE2/2)) ];
    [CircleDataNew,LineDataNew,EP,theta_end,xVec2,yVec2] = discreteClothoid(SE2/2,n,k2,0,EP,theta_end,plotOn);
    xVec = [xVec xVec2]; 
    yVec = [yVec yVec2]; 
    CircleData = [CircleData CircleDataNew]; 
    LineData = [LineData LineDataNew]; 
    Sall = Sall+SE2/2;
    curvArc = [curvArc unique([Sall-SE2/2:1:Sall Sall])];
    curvVec = [curvVec k2/SE2*2*(Sall-unique([Sall-SE2/2:1:Sall Sall])) ];
end