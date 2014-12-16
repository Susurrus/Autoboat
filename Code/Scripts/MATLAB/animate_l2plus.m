% Show a visualization of the L2+ data from a simulated run

% Set to true to save this as a video
saveVideo = true;

% Speed up the playback by 10x by skipping a bunch of timesteps.
speed = 50;
valid_range = 1:speed:length(sensedPosition);

% Grab position data
x = sensedPosition(valid_range, 1);
y = sensedPosition(valid_range, 2);

% And rudder data (for some reason it's negated)
rudder_angle = -rudder_position(valid_range);

% Then grab waypoint and desired path data
from_wp_n = wp0(valid_range, 1);
from_wp_e = wp0(valid_range, 2);
to_wp_n = wp1(valid_range, 1);
to_wp_e = wp1(valid_range, 2);

% Also L2+ vector data
l2_north = L2(valid_range, 1);
l2_east = L2(valid_range, 2);

% And aim-point data
aim_point_n = AimPoint(valid_range, 1);
aim_point_e = AimPoint(valid_range, 2);

% And finally course-over-ground data
cog = sensedHeading(valid_range);

% Clean up the commanded rudder data (for some reason it's negated)
commanded_rudder_angle = -180/pi*commands_rudder_angle(valid_range);

%% Animate (use static_plots to determine run to use)

% Prepare for saving video
if saveVideo
    videoFileName = sprintf('%f.avi', now);
    aviWriter = VideoWriter(videoFileName);
    aviWriter.FrameRate = 30;
    open(aviWriter);
end

% Get the coordinates for icons for the boat and rudder
[boat_coords, rudder_coords] = icon_coords();

% Set whether the animation is playing or not. Setting this to true
% pauses the rendering loop.
isPaused = false;

% If a video is requested, fullscreen the video. Resizing the video
% mid-playback during recording doesn't work, so this is the only way
% to get big videos out. This must be done in the first figure() call
% and not in a separate get() call.
if saveVideo
    f = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
else
    f = figure();
end

% Finally plot everything for the first timepoint
positionAxis = subplot(3,2,[1 3 5]);
axis(positionAxis, 'equal');
hold(positionAxis, 'on');
rudderAxis = subplot(3,2,2);
hold(rudderAxis, 'on');
title(rudderAxis, 'Rudder');
crosstrackErrorAxis = subplot(3,2,4);
hold(crosstrackErrorAxis, 'on');
set(crosstrackErrorAxis, 'xticklabel', []);
title(crosstrackErrorAxis, 'Crosstrack Error (m)');
headingErrorAxis = subplot(3,2,6);
hold(headingErrorAxis, 'on');
set(headingErrorAxis, 'xticklabel', []);
title(headingErrorAxis, 'Heading error');

% Keep the positionAxis current, necessary for patch()
axes(positionAxis);

% Plot the boat position as a dot trail
boat_pos = plot(positionAxis, y(valid_range(1)), x(valid_range(1)), '.', 'MarkerSize', 15);

% Plot the entire waypoint track
path = plot(positionAxis, [from_wp_e to_wp_e], [from_wp_n to_wp_n], 'k-');

% First calculate the position of the boat at all points during this range
boat_rotmat = rotmat2(-cog(valid_range(1)));
boat_coords_rot = boat_coords;
for i = 1:length(boat_coords_rot)
    boat_coords_rot(i,:) = boat_rotmat*boat_coords_rot(i,:)';
end
boat_rotmat = rotmat2(-cog(valid_range(1)));

% And plot the rudder angle
rudder_rotmat = rotmat2(-pi/180*rudder_angle(valid_range(1)));
rudder_coords_rot = rudder_coords;
for i = 1:length(rudder_coords_rot)
    rudder_coords_rot(i,:) = boat_rotmat*rudder_rotmat*rudder_coords_rot(i,:)';
end
rudder = patch(y(valid_range(1)) + boat_coords_rot(8,1) + rudder_coords_rot(:,1), x(valid_range(1)) + boat_coords_rot(8,2) + rudder_coords_rot(:,2), 'y');

% And the boat itself (done after the rudder to get ordering right)
boat = patch(y(valid_range(1)) + boat_coords_rot(:,1), x(valid_range(1)) + boat_coords_rot(:,2), 'y');

% And the current L2+ vector
l2_vector = quiver(positionAxis, y(valid_range(1)), x(valid_range(1)), l2_north(valid_range(1)), l2_east(valid_range(1)));

% And the aim point
aim_point = plot(positionAxis, aim_point_e, aim_point_n, 'b*');

% Highlight the current waypoint pairing
from_wp = plot(positionAxis, from_wp_n(1), from_wp_e(1), 'g*', 'MarkerSize', 10);
to_wp = plot(positionAxis, to_wp_n(1), to_wp_e(1), 'r*', 'MarkerSize', 10);

% And the points of waypoint transition
transitions = plot(0, 0, 'm.');

% On the rudder plot, plot the commanded and real rudder angle
r_angle = plot(rudderAxis, 0, 180/pi*rudder_angle(1), 'k-');
c_r_angle = plot(rudderAxis, 0, commanded_rudder_angle(1), '--b');
set(rudderAxis, 'YLim', [-45 45]);
set(rudderAxis, 'YTick', -45:15:45);
set(rudderAxis, 'YGrid', 'on');

% Save this frame to our video.
if saveVideo
    currentFrame = getframe(f);
    writeVideo(aviWriter, currentFrame);
end

% And now animate this plot by updating the data for every plot element.
for i=2:length(valid_range)
    % If the user closes the window quit animating.
    if ~ishandle(f)
        close(aviWriter);
        return;
    end

    % Wait if the animation is paused
    while isPaused
        if ~ishandle(f)
            close(aviWriter);
            return;
        end
        pause(0.1)
    end

    % Update the boat position
    set(boat_pos, 'XData', y(1:i), 'YData', x(1:i));

    % Update the boat drawing
    boat_rotmat = rotmat2(-cog(i));
    boat_coords_rot = boat_coords;
    for j = 1:length(boat_coords_rot)
        boat_coords_rot(j,:) = boat_rotmat*boat_coords_rot(j,:)';
    end
    set(boat, 'XData', y(i) + boat_coords_rot(:,1), 'YData', x(i) + boat_coords_rot(:,2));

    % Update the rudder drawing
    rudder_rotmat = rotmat2(rudder_angle(i));
    rudder_coords_rot = rudder_coords;
    for j = 1:length(rudder_coords_rot)
        rudder_coords_rot(j,:) = boat_rotmat*rudder_rotmat*rudder_coords_rot(j,:)';
    end
    set(rudder, 'XData', y(i) + boat_coords_rot(8,1) + rudder_coords_rot(:,1), 'YData', x(i) + boat_coords_rot(8,2) + rudder_coords_rot(:,2));

    % Update the viewport, keeping it square and showing: the aim point,
    % from wp, and to wp.
    y_positions = [y(i) aim_point_e(i) y(i) + l2_east(i) from_wp_e(i) to_wp_e(i)];
    y_range = [min(y_positions); max(y_positions)];
    y_range_size = abs(y_range(1) - y_range(2));
    if y_range_size < 60 % Make sure the viewport is at least 60m tall
        extra = (60 - y_range_size) / 2;
        y_range(1) = y_range(1) - extra;
        y_range(2) = y_range(2) + extra;
    end
    x_positions = [x(i) aim_point_n(i) x(i) + l2_north(i) from_wp_n(i) to_wp_n(i)];
    x_range = [min(x_positions); max(x_positions)];
    x_range_size = abs(x_range(1) - x_range(2));
    if x_range_size < 60 % Make sure the viewport is at least 60m wide
        extra = (60 - x_range_size) / 2;
        x_range(1) = x_range(1) - extra;
        x_range(2) = x_range(2) + extra;
    end

    % Now make sure both dimensions are the same, so we have a nice square
    % viewport.
    y_range_size = abs(y_range(1) - y_range(2));
    x_range_size = abs(x_range(1) - x_range(2));
    if y_range_size < x_range_size
        extra = (x_range_size - y_range_size) / 2;
        y_range(1) = y_range(1) - extra;
        y_range(2) = y_range(2) + extra;
    else
        extra = (y_range_size - x_range_size) / 2;
        x_range(1) = x_range(1) - extra;
        x_range(2) = x_range(2) + extra;
    end
    % And keep a 30m buffer around these limits
    buffer = [-30; 30];
    y_range = y_range + buffer;
    x_range = x_range + buffer;
    set(positionAxis, 'XLim', y_range, 'YLim', x_range);

    % Update the L2+ vector
    set(l2_vector, 'XData', y(i), 'YData', x(i), 'UData', l2_east(i), 'VData', l2_north(i));

    % Update the aimpoint
    set(aim_point, 'XData', aim_point_e(i), 'YData', aim_point_n(i));

    % Update the waypoint track as needed
    set(from_wp, 'XData', from_wp_e(i), 'YData', from_wp_n(i));
    set(to_wp, 'XData', to_wp_e(i), 'YData', to_wp_n(i));

    % If there's been a waypoint transition, plot it too
    if from_wp_e(i) ~= from_wp_e(i-1) || from_wp_n(i) ~= from_wp_n(i-1)
        transitions_x = [get(transitions, 'XData') y(i)];
        transitions_y = [get(transitions, 'YData') x(i)];
        set(transitions, 'XData', transitions_x, 'YData', transitions_y);
    end

    % Update the plot title
    title(positionAxis, sprintf('Autonomous run (%d/%d)', i, length(valid_range)));

    % Update the rudder data
    set(r_angle, 'XData', 1:i, 'YData', 180/pi*rudder_angle(1:i));
    set(c_r_angle, 'XData', 1:i, 'YData', commanded_rudder_angle(1:i));

    % And limit the data views to the last 10s of data
    set(rudderAxis, 'XLim', [max(1,i-100*100/speed); i]);

    % Save this frame to our video.
    if saveVideo
        currentFrame = getframe(f);
        writeVideo(aviWriter, currentFrame);

    % Pause until the next frame
    else
        if i < length(sensedPosition)
            pause(.001);
        end
    end
end
close(aviWriter);