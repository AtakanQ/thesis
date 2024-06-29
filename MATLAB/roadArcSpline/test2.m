
folderName = 'ismail\';
roadName = 'I 75';
files = dir(folderName);
counter = 1;
for i = 3:length(files)
    if (contains(files(i).name, roadName) && (files(i).name(1:3)~="nan"))
        roads_lonlat{counter} = readCSV(strcat(folderName,files(i).name));
        counter = counter + 1;
    end
end
refLat = mean(roads_lonlat{1}(:,2));
refLon = mean(roads_lonlat{1}(:,1));

figure;
for j = 1:length(roads_lonlat)
    [xEast, yNorth, zUp] = geodetic2enu(roads_lonlat{j}(:,2), ...
    roads_lonlat{j}(:,1), 0, refLat, refLon, 0, wgs84Ellipsoid);
    plot(xEast,yNorth,'LineWidth',1.2)
    hold on
    axis equal
end
axis equal

figure;
for j = 1:length(roads_lonlat)
    [xEast, yNorth, zUp] = geodetic2enu(roads_lonlat{j}(:,2), ...
    roads_lonlat{j}(:,1), 0, refLat, refLon, 0, wgs84Ellipsoid);
    for k = 1:length(xEast)
        plot(xEast(k),yNorth(k),'.','MarkerSize',15,'Color',[j/length(roads_lonlat) 0 0])
        hold on
    end
    axis equal
end
axis equal

