%% Sun power test.
% Tests our algorithm for determining solar power seen by the boat. All
% angles and axes are based on those in Fossen 1994.

%% Test 1: High-noon Sanity Check
% Result should be 1 as the sun vector maps perfectly onto the boat surface
% normal.

% Sun is at high noon.
azimuth = 0;
zenith = 0;

% Boat is resting in neutral position.
p = 0;
q = 0;

% Transform sun position into cartesian coordinates.
inertial_sun = [cos(azimuth)*sin(zenith); sin(azimuth)*sin(zenith); -cos(zenith)];

% Calculate rotation vector for z-axis.
euler_vec = [-sin(q) cos(q)*sin(p) -cos(q)*cos(p)];

% Should be 1.
sun_power = euler_vec*inertial_sun

%% Test 2: Zero-power Result
% pi/2 angle between sun and boat should result in 0 power calculated. This
% means that 0% of the sun vector maps onto the boat normal.

% Sun is pi/4 to port.
azimuth = -pi/2;
zenith = pi/4;

% Boat listing pi/4 to starboard.
p = pi/4;
q = 0;

% Transform sun position into cartesian coordinates.
inertial_sun = [cos(azimuth)*sin(zenith); sin(azimuth)*sin(zenith); -cos(zenith)];

% Calculate rotation vector for z-axis.
euler_vec = [-sin(q) cos(q)*sin(p) -cos(q)*cos(p)];

% Should be practically 0.
sun_power = euler_vec*inertial_sun

%% Test 3: Full-power Result
% Result should be 1 as we are projecting sun vector onto boat top normal.
% 1 means the sun vector maps perfectly, which means 100% power.

% Sun is pi/3 down from high noon to the north.
azimuth = 0;
zenith = pi/3;

% Boat listing pi/3 down in the bow.
p = 0;
q = -pi/3;

% Transform sun position into cartesian coordinates.
inertial_sun = [cos(azimuth)*sin(zenith); sin(azimuth)*sin(zenith); -cos(zenith)];

% Calculate rotation vector for z-axis.
euler_vec = [-sin(q) cos(q)*sin(p) -cos(q)*cos(p)];

sun_power = euler_vec*inertial_sun
