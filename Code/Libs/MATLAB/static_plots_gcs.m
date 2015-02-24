%% Plot each autonomous run from the provided CSV file.
function static_plots_gcs(data)
    
    % Check that all timestamps are unique, otherwise things explode later
    assert(length(data.timestamp) == length(unique(data.timestamp)), 'Timestamp data must be unique for calculations');
    
    % And let's start our timestamps at 0, which makes the plots a little
    % nicer.
    data.timestamp = data.timestamp - data.timestamp(1);
    
    %% Determine periods of autonomous control
    % Check for both autonomous mode from HEARTBEAT @ 10Hz
    valid_autodata_data = ~isnan(data.HEARTBEAT.base_mode);
    auto_mode = bitand(data.HEARTBEAT.base_mode(valid_autodata_data), 4) ~= 0;
    mode_time = data.timestamp(valid_autodata_data);
    [mode_time, i] = unique(mode_time);
    auto_mode = auto_mode(i);

    % And manual override from NODE_STATUS @ 1Hz for the primary node.
    valid_reset_data = ~isnan(data.NODE_STATUS.primary_errors);
    manual_override = bitand(data.NODE_STATUS.primary_errors(valid_reset_data), hex2dec('10')) ~= 0;
    [override_time, oti] = unique(data.timestamp(valid_reset_data));
    manual_override = manual_override(oti);
    clear oti;

    % Now resample the manual override signals, as that's the slower one
    manual_override_int = interp1(override_time, manual_override, mode_time, 'nearest');
    manual_override_int(isnan(manual_override_int)) = 0;

    % And now find all the times we're in autonomous mode and it's not being
    % overridden.
    automode = auto_mode & ~manual_override_int;

    figure;
    hold on;
    plot(mode_time, automode);
    title('Autonomous control over time');

    %% Plot data for all autonomous mode segments
    % First grab position
    valid_lpos_data = ~isnan(data.LOCAL_POSITION_NED.x);
    assert(any(valid_lpos_data), 'No valid position data found for LOCAL_POSITION_NED.x.');
    north = data.LOCAL_POSITION_NED.x(valid_lpos_data);
    east = data.LOCAL_POSITION_NED.y(valid_lpos_data);
    pos_time = data.timestamp(valid_lpos_data);

    % Then grab waypoint and desired path data
    valid_waypoint_data = ~isnan(data.WAYPOINT_STATUS.last_wp_north);
    from_wp_n = data.WAYPOINT_STATUS.last_wp_north(valid_waypoint_data);
    from_wp_e = data.WAYPOINT_STATUS.last_wp_east(valid_waypoint_data);
    to_wp_n = data.WAYPOINT_STATUS.next_wp_north(valid_waypoint_data);
    to_wp_e = data.WAYPOINT_STATUS.next_wp_east(valid_waypoint_data);
    waypoint_time = data.timestamp(valid_waypoint_data);
    fwaypoint_n_with_pos = interp1(waypoint_time, from_wp_n, pos_time, 'nearest');
    fwaypoint_e_with_pos = interp1(waypoint_time, from_wp_e, pos_time, 'nearest');
    twaypoint_n_with_pos = interp1(waypoint_time, to_wp_n, pos_time, 'nearest');
    twaypoint_e_with_pos = interp1(waypoint_time, to_wp_e, pos_time, 'nearest');

    % Also grab rudder angle data
    valid_basic_state_data = ~isnan(data.BASIC_STATE2.rudder_angle);
    rudder_angle = data.BASIC_STATE2.rudder_angle(valid_basic_state_data);
    commanded_rudder_angle = data.BASIC_STATE2.rudder_angle(valid_basic_state_data);
    basic_state_time = data.timestamp(valid_basic_state_data);

    % And water velocity (m/s) from DST800 @ 2Hz for the primary node.
    valid_water_speed_data = ~isnan(data.DST800.speed);
    water_speed_time = data.timestamp(valid_water_speed_data);
    water_speed = data.DST800.speed(valid_water_speed_data);
    water_speed = interp1(water_speed_time, water_speed, pos_time);
    clear valid_vwater_data vwater_time;

    % And L2+ vector data
    l2_north = data.BASIC_STATE2.L2_north(valid_basic_state_data);
    l2_east = data.BASIC_STATE2.L2_east(valid_basic_state_data);

    % And velocity data
    vel_n = data.LOCAL_POSITION_NED.vx(valid_lpos_data);
    vel_e = data.LOCAL_POSITION_NED.vy(valid_lpos_data);

    % And finally course-over-ground data
    valid_gps_data = ~isnan(data.GPS_RAW_INT.lat) & data.GPS_RAW_INT.fix_type >= 2 & data.GPS_RAW_INT.lat ~= 0 & data.GPS_RAW_INT.lon ~= 0;
    cog = data.GPS_RAW_INT.cog(valid_gps_data);
    gps_time = data.timestamp(valid_gps_data);

    % Interpolate the autonomous mode into the position timeslots
    % 'nearest' mode is used to make sure these values stay logical.
    mode_with_pos = interp1(mode_time, automode, pos_time, 'nearest');
    mode_with_pos(isnan(mode_with_pos)) = 0;
    mode_with_pos = logical(mode_with_pos);
    plot(mode_with_pos)

    % Split this data into groups based on the mode of autonomousity.
    start_indices = find(diff(mode_with_pos) > 0);
    % Make sure that if the nearest-neighbor interpolation selects a
    % timestep where the vehicle was under manual control, correct for it.
    while mode_with_pos(start_indices(1)) == 0
        start_indices(1) = start_indices(1) + 1;
    end
    end_indices = find(diff(mode_with_pos) < 0);
    assert(length(start_indices) == length(end_indices));

    % Output the different run indices. These are all in reference to the
    % timestamps for the LOCAL_POSITION_NED messages.
    segments = [start_indices end_indices]

    % Finally plot all details for all of these segments
    for i=1:length(start_indices)
        figure;
        %subplot(1,2,1);
        hold on;

        valid_range = start_indices(i):end_indices(i);

        % First plot the position of the boat at all points during this range
        plot(east(valid_range), north(valid_range), '.');

        % And make sure the start and end points are clear
        start_point = plot(east(valid_range(1)), north(valid_range(1)), 'g.', 'MarkerSize', 15);
        end_point = plot(east(valid_range(end)), north(valid_range(end)), 'r.', 'MarkerSize', 15);
        legend([start_point end_point], 'Start', 'End');

        % Determine what the waypoints were during this run.
        % unique() returns a sorted list, which is not what we want. So these
        % lines are all there to restore the original sort ordering.
        o = fwaypoint_n_with_pos(valid_range);
        [~,b] = unique(o, 'first');
        b = sort(b);
        a = o(b);
        o = fwaypoint_e_with_pos(valid_range);
        [~,d] = unique(o, 'first');
        d = sort(d);
        c = o(d);
        o = twaypoint_n_with_pos(valid_range);
        [~,f] = unique(o, 'first');
        f = sort(f);
        e = o(f);
        o = twaypoint_e_with_pos(valid_range);
        [~,h] = unique(o, 'first');
        h = sort(h);
        g = o(h);
        
        % Now add any necessary "fake" waypoints into destination waypoint
        % vectors.
        o = twaypoint_n_with_pos(valid_range);
        e = o(b);
        o = twaypoint_e_with_pos(valid_range);
        g = o(b);
        
        % Plot lines between the waypoints and number the waypoints.
        for j=1:length(a)
            plot([c(j);g(j)], [a(j);e(j)], '.--k');
            baseNum = 2*(j-1);
            text(c(j) + 1, a(j) + 1, num2str(baseNum));
            text(g(j) + 1, e(j) + 1, num2str(baseNum + 1));
        end

        % And plot the occurance of any waypoint transitions that occurred.
        wp_transitions = diff(twaypoint_n_with_pos(valid_range)) ~= 0;
        plot(east(valid_range(wp_transitions)), north(valid_range(wp_transitions)), '.m', 'MarkerSize', 15);

        title(sprintf('Autonomous run %d of %.0f seconds', i, pos_time(end_indices(i)) - pos_time(start_indices(i))));
        axis equal;

        % And plot speed data as well.
        %subplot(1,2,2);
        %hold on;
        %plot(pos_time(valid_range), water_speed(valid_range));
        %ylabel('Speed (m/s)');
        %plot(pos_time(valid_range), sqrt(sum(vel_x(valid_range).^2 + vel_y(valid_range).^2, 2)));
        %legend({'Water speed'});
    end
end
