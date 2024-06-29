function [arcVec,xVec,yVec] = plotBiElementaryAcrSpline(CircleData,LineData)

nSeg = max([CircleData(end,:) LineData(end,:)]);
xVec = [0];
yVec = [0];
arcVec = [0];
for jn = 1:nSeg
   colIndex = find(CircleData(end,:) == jn );
   % Circle Segment
   if ~isempty(colIndex)
        currentArc = [0:.01:CircleData(end-1,colIndex)];
        R0 = CircleData(3,colIndex);
        CX = CircleData(1,colIndex);
        CY = CircleData(2,colIndex);
        if R0 > 0
            phi = unique([CircleData(4,colIndex):.0001:CircleData(5,colIndex) CircleData(5,colIndex)]);
            arcVec = [arcVec arcVec(end)+abs(R0*(phi-phi(1)))];
            xVec = [xVec CX+R0*sin(phi)];
            yVec = [yVec CY-R0*cos(phi)];
        else
            phi = fliplr(unique([CircleData(4,colIndex):-.0001:CircleData(5,colIndex) CircleData(5,colIndex)]));
            arcVec = [arcVec arcVec(end)+abs(R0*(phi-phi(1)))];
            xVec = [xVec CX+R0*sin(phi)];
            yVec = [yVec CY-R0*cos(phi)];
        end
   % Line Segment
   else
       colIndex = find(LineData(end,:) == jn );
       lineVec = [0:.01:1];
       xVec = [xVec xVec(end)+lineVec*(LineData(3,colIndex)-LineData(1,colIndex))];
       arcVec = [arcVec arcVec(end)+lineVec*LineData(5,colIndex)];
       yVec = [yVec yVec(end)+lineVec*(LineData(4,colIndex)-LineData(2,colIndex))];
   end
end