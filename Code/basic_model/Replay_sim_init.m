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

%% Determine periods of autonomous control
% Check for both autonomous mode from HEARTBEAT @ 10Hz
valid_autodata_data = ~isnan(data.HEARTBEAT.base_mode);
auto_mode = bitand(data.HEARTBEAT.base_mode(valid_autodata_data), 4) ~= 0;
mode_time = data.timestamp(valid_autodata_data);
[mode_time, i] = unique(mode_time);
auto_mode = auto_mode(i);

%% Now grab only the timesteps where controller data exists
% All the required data is in the CONTROLLER_DATA message
valid_cdata = ~isnan(data.CONTROLLER_DATA.reset);
cdata_time = data.timestamp(valid_cdata);
assert(any(valid_cdata), 'No valid CONTROLLER_DATA messages.');

% System state
ranged_reset = data.CONTROLLER_DATA.reset(valid_cdata);

% Extract the GPS data
ranged_lat = data.CONTROLLER_DATA.lat(valid_cdata);
ranged_lon = data.CONTROLLER_DATA.lon(valid_cdata);
ranged_cog = data.CONTROLLER_DATA.cog(valid_cdata);
ranged_sog = data.CONTROLLER_DATA.sog(valid_cdata);
ranged_gpsNewData = data.CONTROLLER_DATA.new_gps_fix(valid_cdata);

% Then grab waypoint and desired path data
ranged_from_wp_n = data.CONTROLLER_DATA.last_wp_north(valid_cdata);
ranged_from_wp_e = data.CONTROLLER_DATA.last_wp_east(valid_cdata);
ranged_to_wp_n = data.CONTROLLER_DATA.next_wp_north(valid_cdata);
ranged_to_wp_e = data.CONTROLLER_DATA.next_wp_east(valid_cdata);
    
% And actuator data
ranged_rudder = data.CONTROLLER_DATA.rudder_angle(valid_cdata);
ranged_throttle = data.CONTROLLER_DATA.prop_speed(valid_cdata);

% And the generated commands
ranged_commanded_rudder = data.CONTROLLER_DATA.commanded_rudder_angle(valid_cdata);

% And IMU stuff
ranged_yaw = data.CONTROLLER_DATA.yaw(valid_cdata);
ranged_pitch = data.CONTROLLER_DATA.pitch(valid_cdata);
ranged_roll = data.CONTROLLER_DATA.roll(valid_cdata);
ranged_x_angle_vel = data.CONTROLLER_DATA.x_angle_vel(valid_cdata);
ranged_y_angle_vel = data.CONTROLLER_DATA.y_angle_vel(valid_cdata);
ranged_z_angle_vel = data.CONTROLLER_DATA.z_angle_vel(valid_cdata);

% And water velocity (m/s) from DST800 @ 2Hz for the primary node.
ranged_water_speed = data.CONTROLLER_DATA.water_speed(valid_cdata);

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

% Select the proper range to animate
valid_range = start_indices(auto_run):end_indices(auto_run);

%% Now enclose all the valid data into Simulink.Timeseries objects for
% playback.
% First set the timestamps that we're working with
sim_cdata_time = cdata_time(valid_range);
sim_cdata_time = sim_cdata_time - sim_cdata_time(1);
gpsNewData = Simulink.Timeseries;
gpsNewData.Name = 'GPS new data';
gpsNewData.Time = sim_cdata_time;
gpsNewData.Data = logical(ranged_gpsNewData(valid_range));
cog = Simulink.Timeseries;
cog.Name = 'cog';
cog.Time = sim_cdata_time;
cog.Data = uint16(ranged_cog(valid_range));
sog = Simulink.Timeseries;
sog.Name = 'sog';
sog.Time = sim_cdata_time;
sog.Data = uint16(ranged_sog(valid_range));
latitude = Simulink.Timeseries;
latitude.Name = 'latitude';
latitude.Time = sim_cdata_time;
latitude.Data = int32(ranged_lat(valid_range));
longitude = Simulink.Timeseries;
longitude.Name = 'longitude';
longitude.Time = sim_cdata_time;
longitude.Data = int32(ranged_lon(valid_range));

yaw = Simulink.Timeseries;
yaw.Name = 'yaw';
yaw.Time = sim_cdata_time;
yaw.Data = single(ranged_yaw(valid_range) / 8192.0);
pitch = Simulink.Timeseries;
pitch.Name = 'pitch';
pitch.Time = sim_cdata_time;
pitch.Data = single(ranged_pitch(valid_range) / 8192.0);
roll = Simulink.Timeseries;
roll.Name = 'roll_rate';
roll.Time = sim_cdata_time;
roll.Data = single(ranged_roll(valid_range) / 8192.0);
x_rate = Simulink.Timeseries;
x_rate.Name = 'x_angular_velocity';
x_rate.Time = sim_cdata_time;
x_rate.Data = single(ranged_x_angle_vel(valid_range) / 4096.0);
y_rate = Simulink.Timeseries;
y_rate.Name = 'y_angular_velocity';
y_rate.Time = sim_cdata_time;
y_rate.Data = single(ranged_y_angle_vel(valid_range) / 4096.0);
z_rate = Simulink.Timeseries;
z_rate.Name = 'z_angular_velocity';
z_rate.Time = sim_cdata_time;
z_rate.Data = single(ranged_z_angle_vel(valid_range) / 4096.0);
attitude = Simulink.Timeseries;
attitude.Name = 'attitude';
attitude.Time = sim_cdata_time;
attitude.Data = [yaw.Data pitch.Data roll.Data];

rudder = Simulink.Timeseries;
rudder.Name = 'rudder';
rudder.Time = sim_cdata_time;
rudder.Data = single(ranged_rudder(valid_range) / 1e4);
auto_rudder = Simulink.Timeseries;
auto_rudder.Name = 'auto_rudder';
auto_rudder.Time = sim_cdata_time;
auto_rudder.Data = single(ranged_commanded_rudder(valid_range) / 1e4);
throttle = Simulink.Timeseries;
throttle.Name = 'throttle';
throttle.Time = sim_cdata_time;
throttle.Data = int16(ranged_throttle(valid_range));

reset = Simulink.Timeseries;
reset.Name = 'reset';
reset.Time = sim_cdata_time;
reset.Data = logical(ranged_reset(valid_range));

wp0 = Simulink.Timeseries;
wp0.Name = 'wp0';
wp0.Time = sim_cdata_time;
wp0.Data = single([ranged_from_wp_n(valid_range)/10 ranged_from_wp_e(valid_range)/10 zeros(size(ranged_to_wp_e(valid_range)))]);
wp1 = Simulink.Timeseries;
wp1.Name = 'wp1';
wp1.Time = sim_cdata_time;
wp1.Data = single([ranged_to_wp_n(valid_range)/10 ranged_to_wp_e(valid_range)/10 zeros(size(ranged_to_wp_e(valid_range)))]);

water_speed = Simulink.Timeseries;
water_speed.Name = 'water_speed';
water_speed.Time = sim_cdata_time;
water_speed.Data = single(ranged_water_speed(valid_range) / 1e4);
