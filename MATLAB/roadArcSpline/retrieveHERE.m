function [laneBorders,laneCenters] = retrieveHERE(folderName,HEREname,refLat,refLon)
%RETRIEVEHERE Summary of this function goes here
%   Detailed explanation goes here
load(strcat(folderName,'\',HEREname))



distanceThreshold = 1;

for i = 1:size(laneBorderData,2) % for each lane
    start_lat = laneBorderData{1,i}(1,2); % might have to look at other end as well
    start_lon = laneBorderData{1,i}(1,3);

    laneBorder.lats = flip(laneBorderData{1,i}(:,2));
    laneBorder.lons = flip(laneBorderData{1,i}(:,3));

    foundFirstTime = false;
    for j = 2:size(laneBorderData,1)
        for k = 1:size(laneBorderData,2)
            target_lat = laneBorderData{j,k}(1,2);
            target_lon = laneBorderData{j,k}(1,3);
            tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
            if(tempDist < distanceThreshold)
                foundFirstTime = true;
                laneBorder.lats = [laneBorder.lats; laneBorderData{j,k}(:,2)];
                laneBorder.lons = [laneBorder.lons; laneBorderData{j,k}(:,3)];
                start_lat = laneBorderData{j,k}(end,2);
                start_lon = laneBorderData{j,k}(end,3);
                break
            end

            target_lat = laneBorderData{j,k}(end,2);
            target_lon = laneBorderData{j,k}(end,3);
            tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
            if(tempDist < distanceThreshold)
                foundFirstTime = true;
                laneBorder.lats = [laneBorder.lats; flip(laneBorderData{j,k}(:,2))];
                laneBorder.lons = [laneBorder.lons; flip(laneBorderData{j,k}(:,3))];
                start_lat = laneBorderData{j,k}(1,2);
                start_lon = laneBorderData{j,k}(1,3);
                break
            end
        end
        if(~foundFirstTime)
            break % failed to concatenate the first piece
        end
    end
    
    if(~foundFirstTime && (j==2) )% failed to concatenate the first piece
        start_lat = laneBorderData{1,i}(end,2); % might have to look at other end as well
        start_lon = laneBorderData{1,i}(end,3);
    
        laneBorder.lats = laneBorderData{1,i}(:,2);
        laneBorder.lons = laneBorderData{1,i}(:,3);
        for j = 2:size(laneBorderData,1)
            for k = 1:size(laneBorderData,2)
                target_lat = laneBorderData{j,k}(1,2);
                target_lon = laneBorderData{j,k}(1,3);
                tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
                if(tempDist < distanceThreshold)
                    laneBorder.lats = [laneBorder.lats; laneBorderData{j,k}(:,2)];
                    laneBorder.lons = [laneBorder.lons; laneBorderData{j,k}(:,3)];
                    start_lat = laneBorderData{j,k}(end,2);
                    start_lon = laneBorderData{j,k}(end,3);
                    break
                end

                target_lat = laneBorderData{j,k}(end,2);
                target_lon = laneBorderData{j,k}(end,3);
                tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
                if(tempDist < distanceThreshold)
                    laneBorder.lats = [laneBorder.lats; flip(laneBorderData{j,k}(:,2))];
                    laneBorder.lons = [laneBorder.lons; flip(laneBorderData{j,k}(:,3))];
                    start_lat = laneBorderData{j,k}(1,2);
                    start_lon = laneBorderData{j,k}(1,3);
                    break
                end
            end
        end
    end    
    laneBorders(i) = laneBorder;
end

for i = 1:size(laneBorders,2)
    diffArrLats = diff(laneBorders(i).lats);
    diffArrLons = diff(laneBorders(i).lons);
    firstDuplicateIndices = find((diffArrLats == 0) & (diffArrLons == 0)) + 1;
    laneBorders(i).lats(firstDuplicateIndices) = [];
    laneBorders(i).lons(firstDuplicateIndices) = [];
    [laneBorders(i).xEast, laneBorders(i).yNorth]= ...
        geodetic2enu(laneBorders(i).lats,laneBorders(i).lons,0,...
        refLat,refLon,0,wgs84Ellipsoid);
end

for i = 1:size(laneCenterData,2) % for each lane
    laneCenter.lats = [];
    laneCenter.lons = [];
    start_lat = laneCenterData{1,i}(1,2); % might have to look at other end as well
    start_lon = laneCenterData{1,i}(1,3);

    laneCenter.lats = flip(laneCenterData{1,i}(:,2));
    laneCenter.lons = flip(laneCenterData{1,i}(:,3));

    foundFirstTime = false;
    for j = 2:size(laneCenterData,1)
        for k = 1:size(laneCenterData,2)
            target_lat = laneCenterData{j,k}(1,2);
            target_lon = laneCenterData{j,k}(1,3);
            tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
            if(tempDist < distanceThreshold)
                foundFirstTime = true;
                laneCenter.lats = [laneCenter.lats; laneCenterData{j,k}(:,2)];
                laneCenter.lons = [laneCenter.lons; laneCenterData{j,k}(:,3)];
                start_lat = laneCenterData{j,k}(end,2);
                start_lon = laneCenterData{j,k}(end,3);
                break
            end

            target_lat = laneCenterData{j,k}(end,2);
            target_lon = laneCenterData{j,k}(end,3);
            tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
            if(tempDist < distanceThreshold)
                foundFirstTime = true;
                laneCenter.lats = [laneCenter.lats; flip(laneCenterData{j,k}(:,2))];
                laneCenter.lons = [laneCenter.lons; flip(laneCenterData{j,k}(:,3))];
                start_lat = laneCenterData{j,k}(1,2);
                start_lon = laneCenterData{j,k}(1,3);
                break
            end
        end
        if(~foundFirstTime)
            break % failed to concatenate the first piece
        end
    end
    
    if(~foundFirstTime && (j==2) )% failed to concatenate the first piece
        start_lat = laneCenterData{1,i}(end,2); % might have to look at other end as well
        start_lon = laneCenterData{1,i}(end,3);
    
        laneCenter.lats = laneCenterData{1,i}(:,2);
        laneCenter.lons = laneCenterData{1,i}(:,3);
        for j = 2:size(laneCenterData,1)
            for k = 1:size(laneCenterData,2)
                target_lat = laneCenterData{j,k}(1,2);
                target_lon = laneCenterData{j,k}(1,3);
                tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
                if(tempDist < distanceThreshold)
                    laneCenter.lats = [laneCenter.lats; laneCenterData{j,k}(:,2)];
                    laneCenter.lons = [laneCenter.lons; laneCenterData{j,k}(:,3)];
                    start_lat = laneCenterData{j,k}(end,2);
                    start_lon = laneCenterData{j,k}(end,3);
                    break
                end

                target_lat = laneCenterData{j,k}(end,2);
                target_lon = laneCenterData{j,k}(end,3);
                tempDist = distance(start_lat,start_lon,target_lat,target_lon,wgs84Ellipsoid);
                if(tempDist < distanceThreshold)
                    laneCenter.lats = [laneCenter.lats; flip(laneCenterData{j,k}(:,2))];
                    laneCenter.lons = [laneCenter.lons; flip(laneCenterData{j,k}(:,3))];
                    start_lat = laneCenterData{j,k}(1,2);
                    start_lon = laneCenterData{j,k}(1,3);
                    break
                end
            end
        end
    end    
    laneCenters(i) = laneCenter;
end

for i = 1:size(laneCenters,2)
    diffArrLats = diff(laneCenters(i).lats);
    diffArrLons = diff(laneCenters(i).lons);
    firstDuplicateIndices = find((diffArrLats == 0) & (diffArrLons == 0)) + 1;
    laneCenters(i).lats(firstDuplicateIndices) = [];
    laneCenters(i).lons(firstDuplicateIndices) = [];
    [laneCenters(i).xEast, laneCenters(i).yNorth]= ...
        geodetic2enu(laneCenters(i).lats,laneCenters(i).lons,0,...
        refLat,refLon,0,wgs84Ellipsoid);
end
end

