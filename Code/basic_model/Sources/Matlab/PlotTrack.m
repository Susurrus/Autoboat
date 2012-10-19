%% Plot waypoints and the vehicle track
Busses;

figure;
hold on;
axis equal;

% Clean up some variables that may have singleton dimensions:
position = squeeze(position);
velocity = squeeze(velocity);

% Fix velocity + position orientation
if size(velocity, 1) == 3
    velocity = velocity';
end
if size(position, 1) == 3
    position = position';
end

% Keep track of what's on the plot for a legend() call
myLegend = {};

% Plot waypoints (red, green, and blue)
% Here we do some predicate indexing to ignore the -1 values at the end
wp_length = length(test_waypoints);
tmp = zeros(wp_length, 3);
for i=1:wp_length
    tmp(i,:) = double(test_waypoints(i).coordinates(:));
end
plot(tmp(:,2), tmp(:,1), 'r^', 'MarkerSize', 10);
text(tmp(:,2)+10, tmp(:,1)-10, cellstr({int2str((1:wp_length)')}), 'Color', 'r');
myLegend{end + 1} = 'Test waypoints';

% The extra two tracks are offset by the current boat position when it
% reset to properly illustrate where the boat thought the waypoints were.
offsets = position(nextTrack > 0,:);
if size(offsets,1) > 0
    wp_length = length(figure8_waypoints);
    tmp = zeros(wp_length, 3);
    for i=1:wp_length
        tmp(i,:) = double(figure8_waypoints(i).coordinates(:));
    end
    figure8_waypoints_offset = double(tmp) + repmat(offsets(1,:),wp_length,1);
    plot(figure8_waypoints_offset(:,2)', figure8_waypoints_offset(:,1)', 'g^', 'MarkerSize', 10);
    text(figure8_waypoints_offset(:,2)'+10, figure8_waypoints_offset(:,1)'-10, cellstr({int2str((1:wp_length)')}), 'Color', 'g');
    myLegend{end + 1} = 'Figure-8 waypoints';
end
if size(offsets,1) > 1
    wp_length = length(sampling_waypoints);
    tmp = zeros(wp_length, 3);
    for i=1:wp_length
        tmp(i,:) = double(sampling_waypoints(i).coordinates(:));
    end
    sampling_waypoints_offset = double(tmp) + repmat(offsets(2,:),wp_length,1);
    plot(sampling_waypoints_offset(:,2)', sampling_waypoints_offset(:,1)', 'b^', 'MarkerSize', 10);
    text(sampling_waypoints_offset(:,2)'+10, sampling_waypoints_offset(:,1)'-10, cellstr({int2str((1:wp_length)')}), 'Color', 'b');
    myLegend{end + 1} = 'Search waypoints';
end

% Add the boat path
% TODO: This should be changed to iterate through a cells array of the
% various waypoint arrays instead of this hackey code.
title('Boat position');
transitions = find(nextTrack > 0);
switch size(transitions,1)
    case 1
        plot(position(1:transitions(1),2),position(1:transitions(1),1), 'r');
        plot(position(transitions(1)+1:end,2),position(transitions(1)+1:end,1), 'g');
    case 2
        plot(position(1:transitions(1),2),position(1:transitions(1),1), 'r');
        plot(position(transitions(1)+1:transitions(2),2),position(transitions(1)+1:transitions(2),1), 'g');
        plot(position(transitions(2)+1:end,2),position(transitions(2)+1:end,1), 'b');
    case 3
        plot(position(1:transitions(1),2),position(1:transitions(1),1), 'r');
        plot(position(transitions(1)+1:transitions(2),2),position(transitions(1)+1:transitions(2),1), 'g');
        plot(position(transitions(2)+1:transitions(3),2),position(transitions(2)+1:transitions(3),1), 'b');
        plot(position(transitions(3)+1:end,2),position(transitions(3)+1:end,1), 'r');
    otherwise
        plot(position(:,2),position(:,1), 'k');
        myLegend{end + 1} = 'Position';
end
grid on;
%% Add additional decorations

decoration_steps = 1:1000:length(position);

% Display the L2 vectors if we have them. Generally we won't have them for
% HIL runs.
if exist('L2', 'var') && ~isempty(L2)
    quiver(position(decoration_steps,2),position(decoration_steps,1),L2(decoration_steps,2),L2(decoration_steps,1), 0, 'm-');
    myLegend{end + 1} = 'L2 Vectors';
end

% Plot velocity vectors
quiver(position(decoration_steps,2),position(decoration_steps,1),velocity(decoration_steps,2), velocity(decoration_steps,1), 0);
myLegend{end + 1} = 'Velocity Vectors';

legend(myLegend);

hold off;
%% Plot the vehicle commands
figure;
subplot(3,1,1);
plot(commands_rudder_up > 0);
title('Rudder Commands');

subplot(3,1,2);
plot(commands_throttle_data);
title('Throttle Commands');

subplot(3,1,3);
plot(commands_ballast_enable);
title('Ballast Commands');

%% Plot commanded rudder versus actual rudder

figure;
hold on;
plot(rudderAngleCommand * 180/pi, 'k:');
plot(rudder_position * 180/pi);
title('Commanded versus actual rudder angle');
ylabel('Rudder angle (deg)');
set(gca, 'XTick', []);
set(gca, 'YLim', [-45 45]);
xlabel('Time');

%% Plot errors

% First plot the desired heading along with actual heading
a = wp1 - wp0;
exp_heading = atan2(a(:,2), a(:,1)) * 180 / pi;
figure;
hold on;
plot(exp_heading, 'r:');
act_heading = unwrap(truthHeading);
act_heading = act_heading * 180 / pi;
plot(act_heading, 'b');
set(gca, 'XTick', []);
title('Actual versus desired heading');
xlabel('Time');
ylabel('Heading (deg)');
legend({'Desired heading', 'Actual heading'}, 'Location', 'Best');
axis tight;

% Now plot the heading error
figure;
hold on;
ax(1) = subplot(2,1,1);
act_heading = unwrap(truthHeading);
act_heading = act_heading * 180 / pi;
plot(exp_heading - act_heading, 'k');
set(gca, 'XTick', []);
title('Heading error');
ylabel('Error (deg)');
axis tight;

% Output the heading RMS error over the whole thing
er = sum((exp_heading - act_heading) .^ 2);
heading_rms_error = sqrt(er/length(exp_heading))

% Now the RMS error over a leg starting with a 10deg heading deviation is:
range = 7433:10780;
er = sum((exp_heading(range) - act_heading(range)) .^ 2);
leg_heading_rms_error = sqrt(er/length(exp_heading(range)))
 
% Add some limits indicating when waypoints changed
transitions = find(diff(wp0(:,1)) > 0 | diff(wp0(:,2)) > 0 | diff(wp1(:,1)) > 0 | diff(wp1(:,2)) > 0);
ylim = get(gca, 'YLim');
x = 1:length(wp0);
for i = 1:length(transitions)
    line([x(transitions(i)) x(transitions(i))], [ylim(1) ylim(2)], 'Color', 'b', 'LineStyle', '--');
end

% Now cross-track errors. This is calculated by rotating the current
% vehicle position into the local waypoint segment's coordinate frame.
% So while the original vessel coordinates were North/East, the new will be
% Along-track distance/cross-track distance.
ax(2) = subplot(2,1,2);
north = [ones(length(wp0), 1) zeros(length(wp0), 1)];
track = wp1(:, 1:2) - wp0(:, 1:2);
% Determine angle between waypoint track and North vector. This is how much we need to rotate the boat position.
angles = -atan2(track(:,2), track(:,1));
rotmat_components = [cos(angles) -sin(angles) sin(angles)];
vessel_pos = position(:,1:2) - wp0(:,1:2);
new_position = size(position(:,1:2));
for i = 1:length(vessel_pos)
    rotmat = [rotmat_components(i, 1) rotmat_components(i, 2); rotmat_components(i, 3) rotmat_components(i, 1)];
    new_position(i, :) = rotmat * vessel_pos(i, :)';
end
crosstrack_error = new_position(:, 2);
plot(crosstrack_error, 'k');
set(gca, 'XTick', []);
title('Crosstrack error');
xlabel('Time');
ylabel('Error (m)');
axis tight;

% Output the heading RMS error over the whole thing
er = sum(exp_heading .^ 2);
heading_rms_error = sqrt(er/length(crosstrack_error))

% Now the RMS error over a leg starting with a 10deg heading deviation is:
range = 7433:10780;
er = sum((exp_heading(range)) .^ 2);
leg_heading_rms_error = sqrt(er/length(exp_heading(range)))

% Finally add some limits indicating when waypoints changed
transitions = find(diff(wp0(:,1)) > 0 | diff(wp0(:,2)) > 0 | diff(wp1(:,1)) > 0 | diff(wp1(:,2)) > 0);
ylim = get(gca, 'YLim');
x = 1:length(wp0);
for i = 1:length(transitions)
    line([x(transitions(i)) x(transitions(i))], [ylim(1) ylim(2)], 'Color', 'b', 'LineStyle', '--');
end

% Now link the zooming of the plots together
linkaxes(ax, 'x');