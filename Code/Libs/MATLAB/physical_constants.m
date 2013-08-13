%% Contains physical real-world constants. Includes both environmental and model constants

% Known constants
% Distance from mid-keel to mid-rudder (m)
wheelbase = Simulink.Parameter;
wheelbase.Description = 'A gain on how quickly the boat turns. Part of the inverted bicycle model.';
wheelbase.Value = 4;
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