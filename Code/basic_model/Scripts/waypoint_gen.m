% Converts distance, heading pairs to relative positions. Useful for
% generating waypoint positions from Google Earth maps. Converts a 2-col
% array of distance (m) and heading (degrees) into actual positions from
% the origin in meters. Output is in a North, East, Down coordinate frame.

% input = [45 272.32];    

% input = [21 220;
%          40 290;
%          23 56;
%          18.5 140.5;
%         ];

% input = [44 305;
%          120.5 356;
%          75.5 5.5;
%          38.5 329;
%          59 0.5;
%         ];

% input = [40 152;
%          58.5 178;
%          64 187.5;
%          35.5 146.5;
%          58 179;
%          43 97;
%         ];

waypoints = zeros(size(input) + [1 1]);

for i = 1:size(input, 1)
    input(i, 2) = input(i, 2) * pi/180;
    waypoints(i + 1, 1) = int32(waypoints(i, 1) + input(i,1) * cos(input(i, 2)));
    waypoints(i + 1, 2) = int32(waypoints(i, 2) + input(i,1) * sin(input(i, 2)));
end

plot(waypoints(:, 2), waypoints(:, 1));
axis image;