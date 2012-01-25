% First plot the equalized heading and rudder angle command on the same plot
figure;hold on;
plot(unwrap(heading)*180/pi);
plot(rudderAngle*180/pi,'g');
plot(rudderAngleCommand*180/pi,'r');

% Now add markers on the graph where the waypoints switch
waypoint_switches = (find(diff(waypoint1(:,1)) > 0) + 1);
waypoint_switches = [1;waypoint_switches];
labels = cellstr(num2str(waypoint1(waypoint_switches,1:2)));
vline_opts.vpos = 'top';
vline_opts.staircase = false;
vline_opts.rotate = 55;
vline2(waypoint_switches, '-k', labels, vline_opts);

% Clean up.
clear vline_opts waypoint_switches;