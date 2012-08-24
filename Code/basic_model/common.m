%% Some shared constants between simulation and the executable

% Simulation timestep size (s)
T_step = 0.01;

% Initial GPS location
% Stored as radians for a priori computations.
ref_lla = [36.955; -122.002; 0] * pi / 180;
% Stored as an int32 in 1e7 rads and 1e6 meters for dynamic computations.
ref_lla_sense = int32([36.9543845 * 1e7; -122.0018435 * 1e7; 0 * 1e6]);

% Precompute some values for LLA -> LTP conversion
a = 6378137; % Earth semi-major axis in WGS84.
e2 = 0.00669437999014;
v = a / sqrt(1 - e2 * sin(ref_lla(1))^2);
r = v * (1 - e2) / (1 - e2 * sin(ref_lla(1))^2);

% The gain is calculated as normal along with a conversion from 1e-7 degrees to rads
lla_ltp_gain = [r * (pi / 180 / 1e7); v * cos(ref_lla(1)) * (pi / 180 / 1e7); 1e-6];
%latLonAlt = zeros(length(global_pos), 3, 'int32');
%newRef = zeros(length(global_pos), 3, 'int32');
%gains = zeros(length(global_pos), 3, 'double');
%for i=1:length(global_pos)
%    latLonAlt(i, :) = [global_pos(i, :)*1e7, 0];
%    newRef(i, :) = ref_lla_sense;
%    gains(i, :) = lla_ltp_gain;
%end
%diff = double(latLonAlt - newRef);
%localPosition = single(diff .* gains);