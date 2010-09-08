%% Plot waypoints and the vehicle track

figure(1);clf;
hold on;
axis equal;

% Plot waypoints (red is initial waypoint blue are following
plot(waypoints(:,2)', waypoints(:,1)', 'r^', 'MarkerSize', 10);
text(waypoints(:,2)'+10, waypoints(:,1)'-10, cellstr({int2str((1:size(waypoints,1))')}));

% Add the boat path
title('Boat position');
plot(position(:,2),position(:,1));
grid on;
%% Add additional decorations

decoration_steps = 1:1000:length(position);

% Display the L2 vectors
quiver(position(decoration_steps,2),position(decoration_steps,1),L2(decoration_steps,2),L2(decoration_steps,1), 0, 'm-');

% Plot velocity vector
quiver(position(decoration_steps,2),position(decoration_steps,1),velocity(decoration_steps,2), velocity(decoration_steps,1), 0);

legend('Waypoints', 'Boat track', 'L2 vectors', 'Velocity vectors');

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