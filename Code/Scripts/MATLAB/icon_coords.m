%% Generate coordinates for rendering a top-down view of the boat and rudder
% User settings
height = 20; % Height of the vessel
width = 6;   % Width of the vessel

% Generate boat patch coordinates
half_coords = [
    0, height/2;
    (width/2)/14, height/2;
    (width/2)/6, 8*(height/2)/9;
    (width/2)/2, 5*(height/2)/9;
    (width/2), (height/2)/3;
    (width/2), -2*(height/2)/3;
    (width/2)/3, -(height/2);
    0, -(height/2);
];
other_half = [-half_coords(2:end,1) half_coords(2:end,2)]; % Mirror across Y-axis
boat_coords = [half_coords; flipud(other_half)];
clear half_coords other_half;

% Generate rudder patch coordinates
rudder_coords = [
    (width/10), 0;
    (width/10), -(height/4);
    -(width/10), -(height/4);
    -(width/10), 0;
];