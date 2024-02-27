function [laneBorders,laneCenters] = retrieveHERE_v2(folderName,HEREname,refLat,refLon,lats,lons)
%RETRIEVEHERE Summary of this function goes here
%   Detailed explanation goes here
load(strcat(folderName,'\',HEREname))



distanceThreshold = 1;

for i = 1:size(laneBorderData,2) % for each lane
    if( ~isempty(laneBorderData{1,i}))
        start_lat = laneBorderData{1,i}(1,2); % might have to look at other end as well
        start_lon = laneBorderData{1,i}(1,3);
    
        laneBorder.lats = flip(laneBorderData{1,i}(:,2));
        laneBorder.lons = flip(laneBorderData{1,i}(:,3));
    
        foundFirstTime = false;
        for j = 2:size(laneBorderData,1)
            for k = 1:size(laneBorderData,2)
                if( ~isempty(laneBorderData{j,k}))
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
                    if( ~isempty(laneBorderData{j,k}))
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
        end    
        laneBorders(i) = laneBorder;
    end
end

for i = 1:size(laneBorders,2)
    diffArrLats = diff(laneBorders(i).lats);
    diffArrLons = diff(laneBorders(i).lons);
    firstDuplicateIndices = find((diffArrLats == 0) & (diffArrLons == 0)) + 1;
    if(~isempty(firstDuplicateIndices))
        laneBorders(i).lats(firstDuplicateIndices) = [];
        laneBorders(i).lons(firstDuplicateIndices) = [];
    end

    valid_indices =  logical((laneBorders(i).lats < lats(1)) .* (laneBorders(i).lats < lats(2) ).*...
    (laneBorders(i).lons < lons(1) ).* (laneBorders(i).lons < lons(2)));
    if(~isempty(valid_indices))
        laneBorders(i).lats(valid_indices) = [];
        laneBorders(i).lons(valid_indices) = [];
    end
    [laneBorders(i).xEast, laneBorders(i).yNorth]= ...
        geodetic2enu(laneBorders(i).lats,laneBorders(i).lons,0,...
        refLat,refLon,0,wgs84Ellipsoid);
end

for i = 1:size(laneCenterData,2) % for each lane
    if( ~isempty(laneCenterData{1,i}))
        laneCenter.lats = [];
        laneCenter.lons = [];
        start_lat = laneCenterData{1,i}(1,2); % might have to look at other end as well
        start_lon = laneCenterData{1,i}(1,3);
    
        laneCenter.lats = flip(laneCenterData{1,i}(:,2));
        laneCenter.lons = flip(laneCenterData{1,i}(:,3));
    
        foundFirstTime = false;
    
    
        for j = 2:size(laneCenterData,1)
            for k = 1:size(laneCenterData,2)
                if( ~isempty(laneCenterData{j,k}))
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
                    if( ~isempty(laneCenterData{j,k}))
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
        end    
        laneCenters(i) = laneCenter;
    end
end


for i = 1:size(laneCenters,2)
    diffArrLats = diff(laneCenters(i).lats);
    diffArrLons = diff(laneCenters(i).lons);
    firstDuplicateIndices = find((diffArrLats == 0) & (diffArrLons == 0)) + 1;

    if(~isempty(firstDuplicateIndices))
        laneCenters(i).lats(firstDuplicateIndices) = [];
        laneCenters(i).lons(firstDuplicateIndices) = [];
    end
    valid_indices =  logical((laneCenters(i).lats < lats(1)) .* (laneCenters(i).lats > lats(2)) .*...
        (laneCenters(i).lons > lons(1)) .* (laneCenters(i).lons < lons(2)) );
    if(~isempty(valid_indices))
        laneCenters(i).lats(~valid_indices) = [];
        laneCenters(i).lons(~valid_indices) = [];
    end
    [laneCenters(i).xEast, laneCenters(i).yNorth]= ...
        geodetic2enu(laneCenters(i).lats,laneCenters(i).lons,0,...
        refLat,refLon,0,wgs84Ellipsoid);
end
end

