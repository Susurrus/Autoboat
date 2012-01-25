% First plot the equalized heading and rudder angle command on the same plot
figure;hold on;
truthHeading = squeeze(truthHeading);
plot(360+unwrap(truthHeading)*180/pi);
plot(rudder_position*180/pi,'g');
plot(rudderAngleCommand*180/pi,'r');

% Clean up.
clear vline_opts waypoint_switches;