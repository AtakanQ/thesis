function [newLat,newLon] = findLatLon(dx,dy,refLat,refLon)

% Convert reference latitude and longitude to radians
refLatRad = deg2rad(refLat);
refLonRad = deg2rad(refLon);

% Earth's radius in meters
R = 6371000;

% Calculate change in latitude and longitude in radians
deltaLat = dy / R;
deltaLon = dx / (R * cos(refLatRad));

% Calculate new latitude and longitude in radians
newLatRad = refLatRad + deltaLat;
newLonRad = refLonRad + deltaLon;

% Convert new latitude and longitude back to degrees
newLat = rad2deg(newLatRad);
newLon = rad2deg(newLonRad);
end

