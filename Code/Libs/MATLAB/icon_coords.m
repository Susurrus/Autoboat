function [boat_coords, rudder_coords] = icon_coords(scalar)
    %% Generate coordinates for rendering a top-down view of the boat and rudder
    % User settings
    height = 5.8928 * scalar; % Height of the vessel (19'4")
    width = 2.032 * scalar;   % Width of the vessel (6'8")

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