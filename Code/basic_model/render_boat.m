%% Renders the boat at a given point in times
% rudder angle in radians: -1..1 - -1 means hard left
% boat angle in radians: 0..2*pi cw from top
function [] = render_boat(boat_position_x, boat_position_y, rudder_angle, boat_heading)

gcf;
axis([-100 100 -100 100]);
axis('equal'); % Forces Matlab to render plot pixels as square
hold on;       % Needed to render everything to same figure
grid on;       % Render a nice grid

% Define boat, rudder, and path points
boat_x = 5*[-2.1 2.1 2.1 .75 0 -.75 -2.1];
boat_y = 5*[-6 -6 0 3.9 6 3.9 0];
rudder_x = 5*[-.15 .15 .15 -.15];
rudder_y = 5*[-1.5 -1.5 0 0];

% Generate some graphic objects for the path, rudder, and boat
h_rudder = findobj('Tag','rudder');
if isempty(h_rudder) == 1
    h_rudder = fill(rudder_x, rudder_y, 'y');
    set(h_rudder,'Erasemode','Background');
    set(h_rudder,'Tag','rudder');
end
h_boat = findobj('Tag','boat');
if isempty(h_boat) == 1
    h_boat = fill(boat_x, boat_y, 'y');
    set(h_boat,'Erasemode','Background');
    set(h_boat,'Tag','boat');
end

% Define new output points
new_boat_x = boat_x;
new_boat_y = boat_y;
new_rudder_x = rudder_x;
new_rudder_y = rudder_y;

% Perform rudder rotation
tmp_x = new_rudder_x;
tmp_y = new_rudder_y;
new_rudder_x = cos(rudder_angle)*tmp_x - sin(rudder_angle)*tmp_y;
new_rudder_y = sin(rudder_angle)*tmp_x + cos(rudder_angle)*tmp_y;

% Move rudder into proper position with the boat.
new_rudder_y = new_rudder_y - 6*5;

% Perform boat rotation (rotates rudder and boat)
tmp_x = new_boat_x;
tmp_y = new_boat_y;
new_boat_x = cos(boat_heading)*tmp_x + sin(boat_heading)*tmp_y;
new_boat_y = -sin(boat_heading)*tmp_x + cos(boat_heading)*tmp_y;

tmp_x = new_rudder_x;
tmp_y = new_rudder_y;
new_rudder_x = cos(boat_heading)*tmp_x + sin(boat_heading)*tmp_y;
new_rudder_y = -sin(boat_heading)*tmp_x + cos(boat_heading)*tmp_y;

% Perform boat translation (translates rudder and boat)
new_boat_x = boat_position_x + new_boat_x;
new_boat_y = boat_position_y + new_boat_y;

new_rudder_x = boat_position_x + new_rudder_x;
new_rudder_y = boat_position_y + new_rudder_y;

% Plot our boat and rudder polygons on the figure
set(h_boat,'Xdata',new_boat_x,'Ydata',new_boat_y);
set(h_rudder,'Xdata',new_rudder_x,'Ydata',new_rudder_y);

drawnow;