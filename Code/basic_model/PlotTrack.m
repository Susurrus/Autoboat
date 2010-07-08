%% Plot waypoints and the vehicle track

figure(1);clf;
hold on;
axis equal;

% Plot waypoints (red is initial waypoint blue are following
for wp=1:size(waypoints,1)
    plot(waypoints(wp,2), waypoints(wp,1), 'r^');
end

% Add the boat path
title('Boat position');
plot(position(:,2),position(:,1));
grid on;
%% Add additional decorations

decoration_steps = 1:50:length(position);

% Display the L2 vectors
plot (position(decoration_steps,2), position(decoration_steps, 1), 'bo');
quiver(position(decoration_steps,2),position(decoration_steps,1),L2(decoration_steps,2),L2(decoration_steps,1), 0, 'm-');

% Plot velocity vector
quiver(position(decoration_steps,2),position(decoration_steps,1),sin(heading(decoration_steps)), cos(heading(decoration_steps)), 0);

%% Plot the vehicle commands
figure(2);clf;
subplot(3,1,1);
plot(rudder_command);
title('Commanded rudder position');

subplot(3,1,2);
plot(throttle_command);
title('Commanded throttle');

subplot(3,1,3);
plot(ballast_command);
title('Commanded tray angle');