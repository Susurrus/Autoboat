% Plot the actual and simulated vehicle tracks on top of each other
figure;
hold on;
plot(sim_GPS.signals.values(:, 7), sim_GPS.signals.values(:, 8), 'b');
%plot(latitude.Data, longitude.Data, 'k');
axis equal;

% Plot the latitude and longitude outputs in direct comparison
figure;
subplot(2, 1, 1);
hold on;
plot(latitude.Time, latitude.Data, 'k');
plot(sim_GPS.time, sim_GPS.signals.values(:, 7), 'b');

subplot(2, 1, 2);
hold on;
plot(longitude.Time, longitude.Data, 'k.');
plot(sim_GPS.time, sim_GPS.signals.values(:, 8), 'b');

% Also plot the rudder and throttle values for comparison
figure;
subplot(2, 1, 1);
hold on;
plot(throttle.Time, throttle.Data, 'k');
plot(sim_throttle.time, sim_throttle.signals.values, 'b');

subplot(2, 1, 2);
hold on;
plot(rudder.Time, rudder.Data * 180 / pi, 'k');
plot(sim_rudder.time, sim_rudder.signals.values * 180 / pi, 'b');


figure;
hold on;
plot(water_speed.Time, water_speed.Data, 'k');
plot(sim_water_speed.time, sim_water_speed.signals.values, 'b');
