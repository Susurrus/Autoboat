%% Contains physical real-world constants. Includes both environmental and model constants

% Known constants
% Distance from mid-keel to mid-rudder (m)
wheelbase = Simulink.Parameter;
wheelbase.Description = 'A gain on how quickly the boat turns. Part of the inverted bicycle model.';
wheelbase.Value = single(15);
wheelbase.DataType = 'single';
wheelbase.DocUnits = 'm';
wheelbase.RTWInfo.StorageClass = 'ExportedGlobal';
wheelbase.RTWInfo.Alias = 'wheelbase';

% GPS slew limit
% Set a GPS rate limit of ~.01km per timestep. Used by limit_gps_rate() in
% the autonomous controller.
% http://www.offroaders.com/info/tech-corner/reading/GPS-Coordinates.htm
% for details.
gps_leap_rate_limit = Simulink.Parameter;
gps_leap_rate_limit.Description = 'A limit on how fast the GPS can change before its assumed to be bad';
gps_leap_rate_limit.Value = int32(0.0001 * 1e7);
gps_leap_rate_limit.DataType = 'int32';
gps_leap_rate_limit.DocUnits = '1e7 degrees';
gps_leap_rate_limit.RTWInfo.StorageClass = 'ExportedGlobal';
gps_leap_rate_limit.RTWInfo.Alias = 'gps_leap_rate_limit';

% Set the GPS offset relative to the center of rotation of the boat.
% Units are in meters and aligned [X, Y, Z] where X is forward and Y is to
% starboard, and Z is down.
gps_offset = [-2.709, -0.155, 0];