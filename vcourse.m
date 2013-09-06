function [lats, lons, totDist] = vcourse(lat1, lon1, lat2, lon2, steps)
%% [lats, lons] = vcourse(lat1, lon1, lat2, lon2, steps)
% Generate an array for the course using the Vincety formula.
%

% Calculate the starting azimuth angle and total distance travelled.
% angle is in degrees
% distance is in meters
[totDist, a12, ~] = vdist(lat1, lon1, lat2, lon2);

% Convert steps to ft for the vreckon function.
% Calculate the total number of waypoints.
n = length(0:steps:totDist);

% Fill lats and lons arrays with waypoint coordinates.
lats = zeros(1,n);
lons = zeros(1,n);
lats(1) = lat1;
lons(1) = lon1;
i = 1;
for k=steps:steps:totDist
    i = i + 1;
    [lats(i), lons(i)] = vreckon(lats(i-1), lons(i-1),steps, a12);
    [~,a12,~] = vdist(lats(i), lons(i),lat2,lon2);
end
lats = lats';
lons = lons';

    
    
