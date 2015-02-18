%%
% This script preps data for use in the Replay_sim.mdl. It loads a .CSV
% file containing MAVLink messages and filters them to make them playable
% in Simulink. Now the CSV files can be of two types: one where the
% CONTROLLER_DATA message contains all data necessary for playback OR where
% a variety of messages contain the data (specifically NODE_STATUS,
% GPS_RAW_INT, TOKIMEC, BASIC_STATE, WAYPOINT_STATUS, and DST800).
%
% The CONTROLLER_DATA message is preferred and if it's seen in the
% datastream will be used.

%% User-configurable variables here
[fname, pname] = uigetfile('*.csv', 'Select CSV file to replay'); % Select dataset file (CSV file)
if isequal(fname, 0) % If no file was specified, just stop execution.
    return;
end
datafile = fullfile(pname, fname);
auto_run = inputdlg('Autonomous run #', 'Input', 1, {'1'}); % Select which autonomous run within this dataset to animate
if isempty(auto_run) % If no run # was specified, just stop execution.
    return;
end
    auto_run = str2double(auto_run{1});

% Load up various Simulink Busses that we need.
Busses;

% Import some standard variables
common;
physical_constants;

% L2+ variables. We override the variables here so we don't need to touch
% the main constants file.
l2plus;
TStar.Value = single(10);
MaxDwnPthStar.Value = single(1);
tanIntercept.Value = tan(45*pi/180);
switchDistance.Value = single(4);
KPsiDot.Value = single(0.7);
GpsOffsetCorrectionEnable.Value = true;
wheelbase.Value = single(15);

%% Grab some values based on file and autonomous run number
% Acquire data from run
data = ProcessCsvFile(datafile); % Select dataset

% Verify that this datafile is legit. Specifically look for timestamp jumps
% of more than 2s.
assert(~any(diff(data.timestamp) > 2), 'Large timestamp jump encountered. Assuming invalid data.');

% Use the HEARTBEAT message to check for autonomous mode operation. This is
% reasonable because the HEARTBEAT is sent immediately on a mode change,
% but is also regularly transmit at 2Hz.
valid_heartbeat_data = ~isnan(data.HEARTBEAT.base_mode);
automode = bitand(data.HEARTBEAT.base_mode(valid_heartbeat_data), 4) ~= 0; % MAV_MODE_FLAG_AUTO_ENABLED
automode_time = data.timestamp(valid_heartbeat_data);
clear valid_automode_data;

% And get the times when the PrimaryNode is in reset, which generally
% indicates use of the emergency backup controller.
if exist('data.CONTROLLER_DATA', 'var')
    valid_errors_data = ~isnan(data.CONTROLLER_DATA.reset);
    errors = data.CONTROLLER_DATA.reset(valid_errors_data) ~= 0;
else
    valid_errors_data = ~isnan(data.NODE_STATUS.primary_errors);
    errors = data.NODE_STATUS.primary_errors(valid_errors_data) ~= 0;
end
errors_time = data.timestamp(valid_errors_data);
clear valid_errors_data;

% Before we can directly compare the erros and automode fields, we have to
% interpolate them into the same base timesteps.
if length(automode) > length(errors)
    actual_mode_time = automode_time;
    errors = interp1(errors_time, errors, automode_time, 'nearest');
    errors(isnan(errors)) = 0; % Account for possible NaNs at the beginning or end
elseif length(errors) > length(automode)
    actual_mode_time = errors_time;
    automode = interp1(automode_time, automode, errors_time, 'nearest');
    automode(isnan(automode)) = 0; % Account for possible NaNs at the beginning or end
else
    actual_mode_time = automode_time;
end
clear errors_time automode_time;

% And now we can determine the real autonomous mode of the SeaSlug. This
% array will be 1s where it's autonomous and driving and 0 otherwise.
actual_mode = automode & ~errors;

% Split this data into groups based on the mode of autonomousity.
% Make sure that if the dataset starts or stops while the vehicle is
% autonomous, that it's handled correctly.
start_indices = find(diff(actual_mode) > 0);
if actual_mode(1) == 1
    start_indices = [1; start_indices];
end
end_indices = find(diff(actual_mode) < 0);
if actual_mode(end) == 1
    end_indices = [end_indices; length(actual_mode)];
end
assert(length(start_indices) == length(end_indices));

% Now map the indices within the `automode` array back into absolute
% indices.
start_index = start_indices(auto_run);
end_index = end_indices(auto_run);
start_index_abs = find(data.timestamp == actual_mode_time(start_index));
end_index_abs = find(data.timestamp == actual_mode_time(end_index));
assert(~isempty(start_index_abs) & ~isempty(end_index_abs), 'Failed to find the start/end indices for this autonomous run');
clear start_indices end_indices;

% Grab the valid range of any dataset interpolated onto the position data
% timesteps
valid_range = start_index_abs:end_index_abs;
valid_range_time = data.timestamp(valid_range);
valid_range_time = valid_range_time - valid_range_time(1);
clear start_index_abs end_index_abs;

%% Now select the appropriate data and package it for the sim.
% Here we switch between pulling all data from the CONTROLLER_DATA message
% or the individual messages.

if exist('data.CONTROLLER_DATA', 'var')
    error('CONTROLLER_DATA messages not supported yet');
else
    % Get all GPS data and prepare it for importing into the model.
    ranged_fix_type = data.GPS_RAW_INT.fix_type(valid_range);
    ranged_lat = data.GPS_RAW_INT.lat(valid_range);
    ranged_lon = data.GPS_RAW_INT.lon(valid_range);
    ranged_vel = data.GPS_RAW_INT.vel(valid_range);
    ranged_cog = data.GPS_RAW_INT.cog(valid_range);
    valid_gps_data = ~isnan(ranged_lat);
    mode_data = uint8(ranged_fix_type(valid_gps_data));
    gps_time = valid_range_time;
    gps_time = gps_time(valid_gps_data);
    gps_time_with_prelude = [((0:.1:0.9) * gps_time(1))'; gps_time]; % Tack on 10 extra data points so the GPS latches before our dataset begins
    % The newData value needs to also apply to the prelude
    gpsNewDataValue = zeros(length(gps_time_with_prelude) * 2, 1);
    gpsNewDataValue(1:2:end) = 1;
    gpsNewDataTime = zeros(length(gps_time_with_prelude) * 2, 1);
    gpsNewDataTime(1:2:end) = gps_time_with_prelude;
    gpsNewDataTime(2:2:end) = gps_time_with_prelude + T_step / 100;

    % Get the IMU's data as yaw/pitch/roll + yaw_rate/pitch_rate/roll_rate
    ranged_yaw = data.TOKIMEC.yaw(valid_range);
    ranged_pitch = data.TOKIMEC.pitch(valid_range);
    ranged_roll = data.TOKIMEC.roll(valid_range);
    ranged_x_angle_vel = data.TOKIMEC.x_angle_vel(valid_range);
    ranged_y_angle_vel = data.TOKIMEC.y_angle_vel(valid_range);
    ranged_z_angle_vel = data.TOKIMEC.z_angle_vel(valid_range);
    ranged_status = data.TOKIMEC.status(valid_range);
    valid_imu_data = ~isnan(ranged_yaw);
    imu_time = valid_range_time;
    imu_time = imu_time(valid_imu_data);
    imuNewDataValue = zeros(length(imu_time) * 2, 1);
    imuNewDataValue(1:2:end) = 1;
    imuNewDataTime = zeros(length(imu_time) * 2, 1);
    imuNewDataTime(1:2:end) = imu_time;
    imuNewDataTime(2:2:end) = imu_time + T_step / 100;
    yaw_data = single(ranged_yaw(valid_imu_data) / 8192.0);
    pitch_data = single(ranged_pitch(valid_imu_data) / 8192.0);
    roll_data = single(ranged_roll(valid_imu_data) / 8192.0);

    % And convert the IMU's yaw/pitch/roll into a w-x-y-z-format quaternion
    attitude_quat = angle2quat(yaw_data, pitch_data, roll_data);

    % And the system's rudder angle from BASIC_STATE @ 10Hz.
    ranged_rudder = data.BASIC_STATE.rudder_angle(valid_range);
    valid_rudder_data = ~isnan(ranged_rudder);
    rudder_time = valid_range_time(valid_rudder_data);

    % And the system's commanded rudder angle from BASIC_STATE @ 10Hz.
    if isfield(data.BASIC_STATE, 'commanded_auto_rudder_angle')
        ranged_auto_rudder = data.BASIC_STATE.commanded_auto_rudder_angle(valid_range);
    else
        ranged_auto_rudder = data.BASIC_STATE.commanded_rudder_angle(valid_range);
    end
    valid_auto_rudder_data = ~isnan(ranged_auto_rudder);
    autorudder_time = valid_range_time(valid_auto_rudder_data);

    % And the system's prop speed from BASIC_STATE @ 10Hz.
    ranged_throttle = data.BASIC_STATE.prop_speed(valid_range);
    valid_throttle_data = ~isnan(ranged_throttle);
    throttle_time = valid_range_time(valid_throttle_data);

    % And the system's reset status from NODE_STATUS @ 1Hz.
    ranged_reset = data.NODE_STATUS.primary_errors(valid_range);
    valid_reset_data = ~isnan(ranged_reset);
    reset_time = valid_range_time(valid_reset_data);

    % And the from- and to-waypoints
    ranged_from_wp_n = data.WAYPOINT_STATUS.last_wp_north(valid_range);
    ranged_from_wp_e = data.WAYPOINT_STATUS.last_wp_east(valid_range);
    ranged_to_wp_n = data.WAYPOINT_STATUS.next_wp_north(valid_range);
    ranged_to_wp_e = data.WAYPOINT_STATUS.next_wp_east(valid_range);
    valid_waypoint_data = ~isnan(ranged_from_wp_n);
    waypoint_time = valid_range_time(valid_waypoint_data);

    % And water velocity (m/s) from DST800 @ 2Hz.
    ranged_water_speed = data.DST800.speed(valid_range);
    valid_water_speed_data = ~isnan(ranged_water_speed);
    water_speed_time = valid_range_time(valid_water_speed_data);
end

%% Now enclose all the valid data into Simulink.Timeseries objects for
% playback.
gpsNewData = Simulink.Timeseries;
gpsNewData.Name = 'GPS new data';
gpsNewData.Time = gpsNewDataTime;
gpsNewData.Data = logical(gpsNewDataValue);
mode = Simulink.Timeseries;
mode.Name = 'mode';
mode.Time = gps_time;
mode.Data = mode_data;
cog = Simulink.Timeseries;
cog.Name = 'cog';
cog.Time = gps_time;
cog.Data = uint16(ranged_cog(valid_gps_data)*100*pi/180); % Comes in as centi-degrees, but controller expects it in .0001 rads.
sog = Simulink.Timeseries;
sog.Name = 'sog';
sog.Time = gps_time;
sog.Data = uint16(ranged_vel(valid_gps_data));
latitude = Simulink.Timeseries;
latitude.Name = 'latitude';
latitude.Time = gps_time_with_prelude;
lat_data = int32(ranged_lat(valid_gps_data));
latitude.Data = [ones(10,1, 'int32') .* lat_data(1); lat_data];
longitude = Simulink.Timeseries;
longitude.Name = 'longitude';
longitude.Time = gps_time_with_prelude;
lon_data = int32(ranged_lon(valid_gps_data));
longitude.Data = [ones(10,1, 'int32') .* lon_data(1); lon_data];
clear lat_data lon_data;

imuNewData = Simulink.Timeseries;
imuNewData.Name = 'GPS new data';
imuNewData.Time = imuNewDataTime;
imuNewData.Data = logical(imuNewDataValue);
yaw = Simulink.Timeseries;
yaw.Name = 'yaw';
yaw.Time = imu_time;
yaw.Data = yaw_data;
pitch = Simulink.Timeseries;
pitch.Name = 'pitch';
pitch.Time = imu_time;
pitch.Data = pitch_data;
roll = Simulink.Timeseries;
roll.Name = 'roll_rate';
roll.Time = imu_time;
roll.Data = roll_data;
x_rate = Simulink.Timeseries;
x_rate.Name = 'x_angular_velocity';
x_rate.Time = imu_time;
x_rate.Data = single(ranged_x_angle_vel(valid_imu_data) / 4096.0);
y_rate = Simulink.Timeseries;
y_rate.Name = 'y_angular_velocity';
y_rate.Time = imu_time;
y_rate.Data = single(ranged_y_angle_vel(valid_imu_data) / 4096.0);
z_rate = Simulink.Timeseries;
z_rate.Name = 'z_angular_velocity';
z_rate.Time = imu_time;
z_rate.Data = single(ranged_z_angle_vel(valid_imu_data) / 4096.0);
tokimec_status = Simulink.Timeseries;
tokimec_status.Name = 'Tokimec status bits';
tokimec_status.Time = imu_time;
tokimec_status.Data = ranged_status(valid_imu_data);
attitude = Simulink.Timeseries;
attitude.Name = 'attitude';
attitude.Time = imu_time;
attitude.Data = attitude_quat;

rudder = Simulink.Timeseries;
rudder.Name = 'rudder';
rudder.Time = rudder_time;
rudder.Data = single(ranged_rudder(valid_rudder_data));
auto_rudder = Simulink.Timeseries;
auto_rudder.Name = 'auto_rudder';
auto_rudder.Time = autorudder_time;
auto_rudder.Data = single(ranged_auto_rudder(valid_auto_rudder_data));
throttle = Simulink.Timeseries;
throttle.Name = 'throttle';
throttle.Time = throttle_time;
throttle.Data = int16(ranged_throttle(valid_throttle_data));

reset = Simulink.Timeseries;
reset.Name = 'reset';
reset.Time = reset_time;
reset.Data = logical(ranged_reset(valid_reset_data) ~= 0);

wp0 = Simulink.Timeseries;
wp0.Name = 'wp0';
wp0.Time = waypoint_time;
wp0.Data = single([ranged_from_wp_n(valid_waypoint_data) ranged_from_wp_e(valid_waypoint_data) zeros(size(ranged_to_wp_e(valid_waypoint_data)))]);
wp1 = Simulink.Timeseries;
wp1.Name = 'wp1';
wp1.Time = waypoint_time;
wp1.Data = single([ranged_to_wp_n(valid_waypoint_data) ranged_to_wp_e(valid_waypoint_data) zeros(size(ranged_to_wp_e(valid_waypoint_data)))]);

water_speed = Simulink.Timeseries;
water_speed.Name = 'water_speed';
water_speed.Time = water_speed_time;
water_speed.Data = single(ranged_water_speed(valid_water_speed_data));
