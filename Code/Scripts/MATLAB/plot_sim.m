% Show a visualization of the simulation results.

% Don't plot every data point to speed things up
speed = 50;
start = find(globalPosition(:,1) ~= 0, 1); % Don't start plotting data until the GPS has locked on.
valid_range = start:speed:length(sensedPosition);

% Grab position data
north = sensedPosition(valid_range, 1);
east = sensedPosition(valid_range, 2);

% And rudder data (for some reason it's negated)
rudder_angle = rudder_position(valid_range);
rudder_angle_command = commands_rudder_angle(valid_range);

time = tout(valid_range);

% Then grab waypoint and desired path data
from_wp_n = wp0(valid_range, 1);
from_wp_e = wp0(valid_range, 2);
to_wp_n = wp1(valid_range, 1);
to_wp_e = wp1(valid_range, 2);
wps = [from_wp_n from_wp_e to_wp_n to_wp_e];

% Plot the boat position and the waypoints
figure;
subplot(2, 2, [1 3]);
hold on;
plot(east, north, '.');
plot([from_wp_e; to_wp_e], [from_wp_n; to_wp_n], 'k*--');
axis equal;
% Display the controller parameters depending on the algorithm used
if ctrl_algo.Value == 1
    plot_title = sprintf('Local position (Ky=%2.3f,K$\\Psi$=%2.3f,K$\\dot{\\Psi}$=%2.3f)', K_crosstrack.Value, K_course.Value, PD_KPsiDot.Value);
elseif ctrl_algo.Value == 0
    plot_title = sprintf('Local position (T*=%2.3f,$\\gamma$=%2.3f,K$\\dot{\\Psi}$=%2.3f)', TStar.Value, atan(tanIntercept), PD_KPsiDot.Value);
end
title(plot_title, 'interpreter', 'latex');
xlabel('East (m)');
ylabel('North (m)');

% Plot the commands
subplot(2, 2, 2);
hold on;
plot(time, 180/pi*rudder_angle_command, 'b--');
plot(time, 180/pi*rudder_angle, 'k-');
title('Rudder angle command');
legend({'Commanded', 'Actual'});
xlabel('Time (s)');
ylabel('Rudder angle (deg)');

% Plot the crosstrack error
all_wp_to_boat = [north - wps(:,1) east - wps(:,2)];
all_wp_to_wp = wps(:,3:4) - wps(:,1:2);
all_wp_to_wp_norm = sqrt(sum(all_wp_to_wp.^2,2));
all_wp_to_wp_unit = [all_wp_to_wp(:,1) ./ all_wp_to_wp_norm all_wp_to_wp(:,2) ./ all_wp_to_wp_norm];
first_alongtrack_dist = dot(all_wp_to_boat, all_wp_to_wp,2) ./ all_wp_to_wp_norm;
all_alongtrack = [all_wp_to_wp_unit(:,1) .* first_alongtrack_dist all_wp_to_wp_unit(:,2) .* first_alongtrack_dist];
crosstrack_vector = all_wp_to_boat - all_alongtrack;
crosstrack_mag = sqrt(sum(crosstrack_vector.^2,2));
crosstrack_sign = sign(all_wp_to_wp_unit(:,2) .* all_wp_to_boat(:,1) - all_wp_to_wp_unit(:,1) .* all_wp_to_boat(:,2));
crosstrack_error = crosstrack_mag .* crosstrack_sign;

subplot(2, 2, 4);
plot(time, crosstrack_error, 'k');
title('Crosstrack error');
xlabel('Time (s)');
ylabel('Error (m)');