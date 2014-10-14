% Plot the different rudder commands
figure;
hold on;
title('Rudder commands');
plot(command_r.time, command_r.signals.values);
rudder_cmd_actual = data.BASIC_STATE.commanded_auto_rudder_angle(valid_range);
rudder_cmd_actual_indices = ~isnan(rudder_cmd_actual);
rudder_cmd_actual = rudder_cmd_actual(rudder_cmd_actual_indices);
rudder_cmd_time = data.timestamp(valid_range);
rudder_cmd_time = rudder_cmd_time(rudder_cmd_actual_indices);
rudder_cmd_time = rudder_cmd_time - rudder_cmd_time(1);
plot(rudder_cmd_time, rudder_cmd_actual, 'k');
legend({'Simulated r_{cmd}', 'Actual r_{cmd}'});