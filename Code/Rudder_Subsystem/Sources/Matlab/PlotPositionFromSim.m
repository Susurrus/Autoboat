% Plot the local position calculated by both the plant and the controller
figure;
hold on;
axis equal;
title('Local position');
position = squeeze(position);
plot(position(2,:), position(1,:), 'k.', 'MarkerSize', 1)
plot(sensedPosition(:,2), sensedPosition(:,1), 'g.', 'MarkerSize', 1)
legend({'True position', 'Sensed position'}, 'Location', 'NorthWest')
plot(test_waypoints(1:6,2), test_waypoints(1:6,1), 'r^')

% And plot the GPS position as well.
figure;
plot(globalPosition(2:end,2), globalPosition(2:end,1));
axis equal;
title('Global position');