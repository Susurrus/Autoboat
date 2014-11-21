%% Some shared constants between simulation and the executable

% Simulation timestep size (s)
T_step = 0.01;

% Maximum PWM uptime (s)
% This variable is used in the input compare block for scaling the output
% to real-world units. This isn't used in the IC block because that block
% can't or won't evaluate MATLAB workspace variables.
MaxPwmIn = .0025;

% Initial GPS location
% Stored as radians for a priori computations.
ref_lla = [36.963765; -122.00191; 0];
ref_lla_rad = ref_lla * pi / 180;

% Stored as an int32 in 1e7 rads and 1e6 meters for dynamic computations.
% This is set as a Simulink.Parameter so it can be exported as a global
% variable in compiled C code.
gpsOrigin = Simulink.Parameter;
gpsOrigin.Description = 'The reference GPS location for this vehicle.';
gpsOrigin.Value = ref_lla .* [1e7; 1e7; 1e6];
gpsOrigin.DataType = 'int32';
gpsOrigin.DocUnits = '1e-7deg,1e-7deg,1e-6m';
gpsOrigin.RTWInfo.StorageClass = 'ExportedGlobal';
gpsOrigin.RTWInfo.Alias = 'gpsOrigin';

% Precompute some values for LLA -> LTP conversion
a = 6378137; % Earth semi-major axis in WGS84.
e2 = 0.00669437999014;
v = a / sqrt(1 - e2 * sin(ref_lla_rad(1))^2);
r = v * (1 - e2) / (1 - e2 * sin(ref_lla_rad(1))^2);

% The gain is calculated as normal along with a conversion from 1e-7 degrees to rads
lla_ltp_gain = [r * (pi / 180 / 1e7); v * cos(ref_lla_rad(1)) * (pi / 180 / 1e7); 1e-6];

% The late ncy in the GPS unit (in units of T_step)
gpsLatency = 150;