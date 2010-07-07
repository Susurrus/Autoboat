%% Plot waypoints and the vehicle track

figure(1);clf;
hold on;
axis equal;

% Plot waypoints (red is initial waypoint blue are following
plot(0, 0, 'r^');
plot(waypoint_1_east, waypoint_1_north, 'r^');

% Add the boat path
title('Boat position');
plot(east, north);
grid on;
%% Add additional decorations

% Display the L2 vectors
for i=1:20:length(north)
    plot (east(i), north(i), 'bo');
    plot ([east(i) east(i)+L2(i,2)], [north(i) north(i)+L2(i,1)], 'm-');
end

for i=1:20:length(velocity)
    plot ([east(i) east(i)+velocity(i,2)], [north(i) north(i)+velocity(i,1)], 'g-');
end


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