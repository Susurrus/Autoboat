%% User-configurable variables here
[fname, pname] = uigetfile('*.csv', 'Select CSV file to replay'); % Select dataset file (CSV file)
datafile = fullfile(pname, fname);
auto_run = inputdlg('Autonomous run #', 'Input', 1, {'1'}); % Select which autonomous run within this dataset to animate
if ~isempty(auto_run)
    auto_run = str2double(auto_run{1});
else
    error('Invalid run # specified')
end

% Load up various Simulink Busses that we need.
Busses;

% Import some standard variables
common;
physical_constants;

% L2+ variables
TStar = single(10);
IPStar = single(0);
InitialPoint = single(0);
Turn2Track = false;
MaxDwnPthStar = single(1);
tanIntercept = tan(45*pi/180);
switchDistance = single(4);
KPsiDot = single(0);
wheelbase.Value = single(4);
cogDotDerivativeDelay = single(0.8);
T_step = 0.01;

%% Grab some values based on file and autonomous run number
% Acquire data from run
data = ProcessCsvFile(datafile); % Select dataset

% Verify that this datafile is legit. Specifically look for timestamp jumps
% of more than 2s.
assert(~any(diff(data.timestamp) > 2), 'Large timestamp jump encountered. Assuming invalid data.');

% Check for both autonomous mode from NODE_STATUS @ 1Hz
valid_automode_data = ~isnan(data.NODE_STATUS.primary_status);
automode = bitand(data.NODE_STATUS.primary_status(valid_automode_data), 1) ~= 0;
automode_time = data.timestamp(valid_automode_data);
clear valid_automode_data;

% And manual override from NODE_STATUS @ 1Hz for the primary node. But here
% we just check for any reset signal, as they will all prevent the vehicle
% from outputting commands to the actuators.
valid_errors_data = ~isnan(data.NODE_STATUS.primary_errors);
errors = data.NODE_STATUS.primary_errors(valid_errors_data) ~= 0;
errors_time = data.timestamp(valid_errors_data);
clear valid_reset_data;

% And now find all the times we're in autonomous mode and not in an error
% state.
automode = automode & ~errors;
clear manual_override_int;

% Split this data into groups based on the mode of autonomousity.
start_indices = find(diff(automode) > 0);
end_indices = find(diff(automode) < 0);
assert(length(start_indices) == length(end_indices));
clear mode_with_pos;

% Now map the indices within the `automode` array back into absolute
% indices.
start_index = start_indices(auto_run);
end_index = end_indices(auto_run);
start_index_abs = find(data.timestamp == automode_time(start_index));
end_index_abs = find(data.timestamp == automode_time(end_index));
assert(~isempty(start_index_abs) & ~isempty(end_index_abs), 'Failed to find the start/end indices for this autonomous run');

% Grab the valid range of any dataset interpolated onto the position data
% timesteps
valid_range = start_index_abs:end_index_abs;
valid_range_time = data.timestamp(valid_range);
valid_range_time = valid_range_time - valid_range_time(1);
clear start_indices end_indices;

%% Now select the appropriate data and package it for the sim.

% Get all GPS data and prepare it for importing into the model.
ranged_fix_type = data.GPS_RAW_INT.fix_type(valid_range);
ranged_lat = data.GPS_RAW_INT.lat(valid_range);
ranged_lon = data.GPS_RAW_INT.lon(valid_range);
ranged_alt = data.GPS_RAW_INT.alt(valid_range);
ranged_eph = data.GPS_RAW_INT.eph(valid_range);
ranged_epv = data.GPS_RAW_INT.epv(valid_range);
ranged_vel = data.GPS_RAW_INT.vel(valid_range);
ranged_cog = data.GPS_RAW_INT.cog(valid_range);
valid_gps_data = ~isnan(ranged_lat);
gps_time = valid_range_time;
gps_time = gps_time(valid_gps_data);
mode = Simulink.Timeseries;
mode.Name = 'mode';
mode.Time = gps_time;
mode.Data = uint8(ranged_fix_type(valid_gps_data));
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
latitude.Time = gps_time;
latitude.Data = int32(ranged_lat(valid_gps_data));
longitude = Simulink.Timeseries;
longitude.Name = 'longitude';
longitude.Time = gps_time;
longitude.Data = int32(ranged_lon(valid_gps_data));

% And the system's rudder angle from BASIC_STATE @ 10Hz.
ranged_rudder = data.BASIC_STATE.rudder_angle(valid_range);
valid_rudder_data = ~isnan(ranged_rudder);
rudder_time = valid_range_time(valid_rudder_data);
rudder = Simulink.Timeseries;
rudder.Name = 'rudder';
rudder.Time = rudder_time;
rudder.Data = single(ranged_rudder(valid_rudder_data));

% And the system's commanded rudder angle from BASIC_STATE @ 10Hz.
if isfield(data.BASIC_STATE, 'commanded_auto_rudder_angle')
    ranged_auto_rudder = data.BASIC_STATE.commanded_auto_rudder_angle(valid_range);
else
    ranged_auto_rudder = data.BASIC_STATE.commanded_rudder_angle(valid_range);
end
valid_auto_rudder_data = ~isnan(ranged_auto_rudder);
autorudder_time = valid_range_time(valid_auto_rudder_data);
auto_rudder = Simulink.Timeseries;
auto_rudder.Name = 'auto_rudder';
auto_rudder.Time = autorudder_time;
auto_rudder.Data = single(ranged_auto_rudder(valid_auto_rudder_data));

% And the system's prop speed from BASIC_STATE @ 10Hz.
ranged_throttle = data.BASIC_STATE.prop_speed(valid_range);
valid_throttle_data = ~isnan(ranged_throttle);
throttle_time = valid_range_time(valid_throttle_data);
throttle = Simulink.Timeseries;
throttle.Name = 'throttle';
throttle.Time = throttle_time;
throttle.Data = int16(ranged_throttle(valid_throttle_data));

% And the system's reset status from NODE_STATUS @ 1Hz.
ranged_reset = data.NODE_STATUS.primary_errors(valid_range);
valid_reset_data = ~isnan(ranged_reset);
reset_time = valid_range_time(valid_reset_data);
reset = Simulink.Timeseries;
reset.Name = 'reset';
reset.Time = reset_time;
reset.Data = logical(ranged_reset(valid_reset_data) ~= 0);

% And the from- and to-waypoints
ranged_from_wp_n = data.WAYPOINT_STATUS.last_wp_north(valid_range);
ranged_from_wp_e = data.WAYPOINT_STATUS.last_wp_east(valid_range);
ranged_to_wp_n = data.WAYPOINT_STATUS.next_wp_north(valid_range);
ranged_to_wp_e = data.WAYPOINT_STATUS.next_wp_east(valid_range);
valid_waypoint_data = ~isnan(ranged_from_wp_n);
waypoint_time = valid_range_time(valid_waypoint_data);
wp0 = Simulink.Timeseries;
wp0.Name = 'wp0';
wp0.Time = waypoint_time;
wp0.Data = single([ranged_from_wp_n(valid_waypoint_data) ranged_from_wp_e(valid_waypoint_data) zeros(size(ranged_to_wp_e(valid_waypoint_data)))]);
wp1 = Simulink.Timeseries;
wp1.Name = 'wp1';
wp1.Time = waypoint_time;
wp1.Data = single([ranged_to_wp_n(valid_waypoint_data) ranged_to_wp_e(valid_waypoint_data) zeros(size(ranged_to_wp_e(valid_waypoint_data)))]);

% And water velocity (m/s) from DST800 @ 2Hz.
ranged_water_speed = data.DST800.speed(valid_range);
valid_water_speed_data = ~isnan(ranged_water_speed);
water_speed_time = valid_range_time(valid_water_speed_data);
water_speed = Simulink.Timeseries;
water_speed.Name = 'water_speed';
water_speed.Time = water_speed_time;
water_speed.Data = single(ranged_water_speed(valid_water_speed_data));
