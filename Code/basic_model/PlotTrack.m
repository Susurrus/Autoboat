%% Plot waypoints and the vehicle track

figure(1);clf;
hold on;
axis equal;

% Plot waypoints (red, green, and blue)
plot(test_waypoints(:,2)', test_waypoints(:,1)', 'r^', 'MarkerSize', 10);
text(test_waypoints(:,2)'+10, test_waypoints(:,1)'-10, cellstr({int2str((1:size(test_waypoints,1))')}), 'Color', 'r');

% The extra two tracks are offset by the current boat position when it
% reset to properly illustrate where the boat thought the waypoints were.
offsets = position(nextTrack > 0,:);
if size(offsets,1) > 0
    figure8_waypoints_offset = figure8_waypoints + repmat(offsets(1,:),size(figure8_waypoints,1),1);
    plot(figure8_waypoints_offset(:,2)', figure8_waypoints_offset(:,1)', 'g^', 'MarkerSize', 10);
    text(figure8_waypoints_offset(:,2)'+10, figure8_waypoints_offset(:,1)'-10, cellstr({int2str((1:size(figure8_waypoints_offset,1))')}), 'Color', 'g');
end
if size(offsets,1) > 1
    sampling_waypoints_offset = sampling_waypoints + repmat(offsets(2,:),size(sampling_waypoints,1),1);
    plot(sampling_waypoints_offset(:,2)', sampling_waypoints_offset(:,1)', 'b^', 'MarkerSize', 10);
    text(sampling_waypoints_offset(:,2)'+10, sampling_waypoints_offset(:,1)'-10, cellstr({int2str((1:size(sampling_waypoints_offset,1))')}), 'Color', 'b');
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
end
grid on;
%% Add additional decorations

decoration_steps = 1:1000:length(position);

% Display the L2 vectors
quiver(position(decoration_steps,2),position(decoration_steps,1),L2(decoration_steps,2),L2(decoration_steps,1), 0, 'm-');

% Plot velocity vector
quiver(position(decoration_steps,2),position(decoration_steps,1),velocity(decoration_steps,2), velocity(decoration_steps,1), 0);

legend('Test waypoints', 'Figure 8 waypoints', 'Search waypoints', 'Boat position', 'L2 vectors', 'Velocity vectors');

hold off;
%% Plot the vehicle commands
figure(2);clf;
subplot(3,1,1);
plot(rudder_command);
title('Rudder Commands');

subplot(3,1,2);
plot(throttle_command);
title('Throttle Commands');

subplot(3,1,3);
plot(ballast_command);
title('Ballast Commands');