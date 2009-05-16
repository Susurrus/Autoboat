%% Generating a boat 2D model
% Each unit is a meter
% Plots results from boat simulation. Running the boatsim and then this
% file works directly. To inject artificial results generate 1D arrays
% rudder_angles, boat_headings, boat_positions_x, boat_positions, y.

figure
axis([-300 300 -300 300]);
axis('equal'); % Forces Matlab to render plot pixels as square
hold on;       % Needed to render everything to same figure
grid on;       % Render a nice grid

% Inputs
% rudder angle in radians: -1..1 - -1 means hard left
% boat angle in radians: 0..2*pi cw from top
%rudder_angles = 1;
%boat_headings = 0;
%boat_positions_x = -50;
%boat_positions_y = 18;

% Define boat, rudder, and path points
boat_x = 50*[-2.1 2.1 2.1 .75 0 -.75 -2.1];
boat_y = 50*[-6 -6 0 3.9 6 3.9 0];
rudder_x = 50*[-.15 .15 .15 -.15];
rudder_y = 50*[-1.5 -1.5 0 0];
path_x = zeros(size(boat_positions_x));
path_y = zeros(size(boat_positions_y));

% Generate some graphic objects for the path, rudder, and boat
h_path = plot(path_x, path_y);
h_rudder = fill(rudder_x, rudder_y, 'y');
set(h_rudder,'Erasemode','Normal');
h_boat = fill(boat_x, boat_y, 'y');
set(h_boat,'Erasemode','Normal');

for t=1:size(boat_positions_x)
    
    title(['Boat simulation at time: ' int2str(t)]);
    
    % Grab the current system measurements for this timestep
    rudder_angle = rudder_angles(t);
    boat_heading = pi/2-boat_headings(t);
    boat_position_x = boat_positions_x(t);
    boat_position_y = boat_positions_y(t);
    
    % Define new output points
    new_boat_x = boat_x;
    new_boat_y = boat_y;
    new_rudder_x = rudder_x;
    new_rudder_y = rudder_y;

    % Perform rudder rotation
    tmp_x = new_rudder_x;
    tmp_y = new_rudder_y;
    new_rudder_x = cos(rudder_angle)*tmp_x + sin(rudder_angle)*tmp_y;
    new_rudder_y = -sin(rudder_angle)*tmp_x + cos(rudder_angle)*tmp_y;

    % Move rudder into proper position with the boat.
    new_rudder_y = new_rudder_y - 6*50;

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
    
    % Update our path
    path_x(t) = boat_position_x;
    path_y(t) = boat_position_y;

    % Plot our boat and rudder polygons on the figure
    set(h_path,'Xdata',path_x,'Ydata',path_y);
    set(h_boat,'Xdata',new_boat_x,'Ydata',new_boat_y);
    set(h_rudder,'Xdata',new_rudder_x,'Ydata',new_rudder_y);
    
    drawnow;
    pause(.01);
end