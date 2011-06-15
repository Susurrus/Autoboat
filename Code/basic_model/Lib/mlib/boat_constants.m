%% Declare boat constants

% Initial conditions

% Initial latitude & longitude. This provides the baseline for the plant
% to generate GPS data.
initial_LL = [36.80611 -121.79639];
v_0 = 0;                % Initial speed
battery_tray_angle = 0; % Initial battery tray angle
T_step = 0.01;          % Simulation timestep
initial_charge = 2851200;    % Initial boat charge, 60% of 6V battery rail (J)
start_time = clock;
%start_time =   1279069200; % 6/13/2010 @ 20:00pm UTC (12pm pacific)
%start_time = 1271205603;% 04/13/2010 @ 7:40pm (TODO: Change to MATLAB commands to capture current time)

% Physical input/output parameters



% Waypoint stuff
% Waypoints are all defined within a local tangent plane in meters North,
% East, Down.
test_waypoints = int32([
             0   0   0;
             450 100 0;
             600 600 0;
             150 300 0;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
            ]);
% Figure eight
figure8_waypoints = int32([
             0   0   0;
             30  60  0;
             0   90  0;
             -30 60  0;
             0   0   0;
             30  -60 0;
             0   -90 0;
             -30 -60 0;
             -1  -1  -1;
             -1  -1  -1;
             -1  -1  -1;
            ]);
% Sampling pattern
sampling_waypoints = int32([
             0    0    0;
             0    210  0;
             -30  210  0;
             -30  0    0;
             -60  0    0;
             -60  210  0;
             -90  210  0;
             -90  0    0;
             -120 0    0;
             -120 210  0;
             -1   -1   -1;
            ]);

% Known constants

m_battery = 108.86;     % Weight of ballast batteries (kg)
d_battery = .3048;      % Distance of batteries from rotation axis (m)
disp_vessel = 612.35;   % Vessel displacement:1350 lbs (kg)
max_batt_angle = 1.309; % Maximum angle for the battery tray: 75 degrees(radians)
max_sea_state = 12;     % The maximum value of the Beaufort Scale
rudder_angle_max = 1;   % Maximum angle of the rudder (radians)
throttle_max = 1;       % Maximum throttle value (% of maximum amps)

% Experimental constants

cg_x = .0508;           % Center of gravity relative to boat center: 1/6' (m)
cg_z = .0508;           % Center of gravity relative to boat center: 1/6' (m)
% Boat center is at 6.5' by the waterline by the axis of symmetry
rudder_angle_dot_max = 10;   % Maximum rate of change of rudder angle (radians/s)
throttle_dot_max = 1;   % Maximum rate of change of throttle (%/s)
proportional_band = .5;      % Proportional band of the rudder response (radians)
roll_period = 3;        % Periodicity of roll. (s)
power_coefficients = [5.6667 9.6667 0]; % Power coefficients for use in the motor current draw calculations
