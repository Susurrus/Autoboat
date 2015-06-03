%% Plot each autonomous run from the provided CSV file.
function static_plots_cdata(data)
    
    %% Determine periods of autonomous control
    % Check for both autonomous mode from HEARTBEAT @ 10Hz
    valid_autodata_data = ~isnan(data.HEARTBEAT.base_mode);
    auto_mode = bitand(data.HEARTBEAT.base_mode(valid_autodata_data), 4) ~= 0;
    mode_time = data.timestamp(valid_autodata_data);
    [mode_time, i] = unique(mode_time);
    auto_mode = auto_mode(i);

    figure;
    hold on;
    plot(mode_time, auto_mode);
    title('Autonomous control over time');

    %% Gather necessary data
    % Get the position data
    valid_cdata = ~isnan(data.CONTROLLER_DATA.reset);
    cdata_time = data.timestamp(valid_cdata);
    assert(any(valid_cdata), 'No valid CONTROLLER_DATA messages.');
    north = data.CONTROLLER_DATA.north(valid_cdata) / 1e3;
    east = data.CONTROLLER_DATA.east(valid_cdata) / 1e3;

    % Then grab waypoint and desired path data
    from_wp_n = data.CONTROLLER_DATA.last_wp_north(valid_cdata) / 10;
    from_wp_e = data.CONTROLLER_DATA.last_wp_east(valid_cdata) / 10;
    to_wp_n = data.CONTROLLER_DATA.next_wp_north(valid_cdata) / 10;
    to_wp_e = data.CONTROLLER_DATA.next_wp_east(valid_cdata) / 10;
    wps = [from_wp_n from_wp_e to_wp_n to_wp_e];
    
    %% Separate the above data into the different autonomous runs

    % Interpolate the autonomous mode into the position timeslots
    % 'nearest' mode is used to make sure these values stay logical.
    mode_with_cdata = interp1(mode_time, auto_mode, cdata_time, 'nearest');
    mode_with_cdata(isnan(mode_with_cdata)) = 0;
    mode_with_cdata = logical(mode_with_cdata);

    % Split this data into groups based on the mode of autonomousity.
    start_indices = find(diff(mode_with_cdata) > 0);
    % Make sure that if the nearest-neighbor interpolation selects a
    % timestep where the vehicle was under manual control, correct for it.
    while mode_with_cdata(start_indices(1)) == 0
        start_indices(1) = start_indices(1) + 1;
    end
    end_indices = find(diff(mode_with_cdata) < 0);
    assert(length(start_indices) == length(end_indices));

    % Output the different run indices. These are all in reference to the
    % timestamps for the CONTROLLER_DATA messages.
    segments = [start_indices end_indices]

    %% Finally plot all details for all of these segments
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
        
        % Plot lines between the waypoints and number the waypoints.
        unique_wps = unique_no_sort_rows(wps(valid_range,:));
        for j=1:size(unique_wps,1)
            plot(unique_wps(j,2:2:4), unique_wps(j,1:2:3), '.--k');
            baseNum = 2*(j-1);
            text(unique_wps(j,2) + 1, unique_wps(j,1) + 1, num2str(baseNum));
            text(unique_wps(j,4) + 1, unique_wps(j,3) + 1, num2str(baseNum + 1));
        end

        % And plot the occurance of any waypoint transitions that occurred.
        wp_transitions = any(diff(wps(valid_range, :)) ~= 0, 2);
        plot(east(valid_range(wp_transitions)), north(valid_range(wp_transitions)), '.m', 'MarkerSize', 15);

        title(sprintf('Autonomous run %d of %.0f seconds', i, cdata_time(end_indices(i)) - cdata_time(start_indices(i))));
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
