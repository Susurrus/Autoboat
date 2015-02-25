% Plot the different rudder commands
figure;
hold on;
title('Rudder commands');

% Plot the actual commanded rudder
plot(auto_rudder.Time, auto_rudder.Data * 180 / pi, 'b.-');

% Plot the actual rudder angle
plot(rudder.Time, rudder.Data * 180 / pi, 'r--');

% Plot the simulated commanded angle
sim_commanded_angle = squeeze(command_r);
plot(tout, sim_commanded_angle * 180 / pi, 'k.-');

ylabel('Rudder angle (deg');
xlabel('Time (s)');
legend({'Actual commmanded', 'Actual angle', 'Sim commanded'});