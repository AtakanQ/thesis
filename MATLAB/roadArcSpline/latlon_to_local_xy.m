function [x, y] = latlon_to_local_xy(lat, lon, refLat, refLon)
    % Convert latitude and longitude from degrees to radians
    lat_rad = deg2rad(lat);
    lon_rad = deg2rad(lon);
    refLat_rad = deg2rad(refLat);
    refLon_rad = deg2rad(refLon);
    
    % Earth's radius in meters
    R = 6371000; % mean radius of the Earth in meters
    
    % Calculate local Cartesian coordinates in meters
    x = R * cos(lat_rad) .* cos(lon_rad - refLon_rad);
    y = R * cos(lat_rad) .* sin(lon_rad - refLon_rad);
    
    % Adjust for the reference latitude
    y = R * (sin(lat_rad - refLat_rad) + sin(refLat_rad)) + y;
end