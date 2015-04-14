% Show a visualization of the simulation results.

% Don't plot every data point to speed things up
speed = 50;
start = find(globalPosition(:,1) ~= 0, 1); % Don't start plotting data until the GPS has locked on.
valid_range = start:speed:length(sensedPosition);

% Grab position data
x = sensedPosition(valid_range, 1);
y = sensedPosition(valid_range, 2);

% And rudder data (for some reason it's negated)
rudder_angle = rudder_position(valid_range);
rudder_angle_command = commands_rudder_angle(valid_range);

% Then grab waypoint and desired path data
from_wp_n = wp0(valid_range, 1);
from_wp_e = wp0(valid_range, 2);
to_wp_n = wp1(valid_range, 1);
to_wp_e = wp1(valid_range, 2);

% Plot the boat position and the waypoints
figure;
hold on;
plot(y, x, '.');
plot([from_wp_e to_wp_e], [from_wp_n to_wp_n], 'k*-');
axis equal;
title('Local position');
xlabel('East (m)');
ylabel('North (m)');

% Plot the errors
%figure;
%subplot(2,1,1);
%subplot(2,1,2);

% Plot the commands
figure;
subplot(2,1,1);
hold on;
plot(180/pi*rudder_angle_command, 'r-');
plot(180/pi*rudder_angle, 'k-');
title('Rudder angle command');
legend({'Commanded', 'Actual'});
xlabel('Time (s)');
ylabel('Rudder angle (deg)');