%% Declare boat constants

% Initial conditions

lat_0 = 36.80611*pi/180;       % Initial latitude (radians)
long_0 = -121.79639*pi/180;    % Initial longitude (radians)
phi_0 = 0;              % Initial heading (eastward positive from north)
v_0 = 0;                % Initial speed
T_step = 0.01;          % Simulation timestep
battery_tray_angle = 0; % Battery tray angle
initial_charge = 2851200;    % Initial boat charge, 60% of 6V battery rail (J)
start_time = 1271205603;% 04/13/2010 @ 7:40pm (TODO: Change to MATLAB commands to capture current time)

% Waypoint stuff
waypoint_1_north = 450;
waypoint_1_east = 100;
waypoint_1_lat = 36.807*pi/180;
waypoint_1_long = -121.795*pi/180;

% Known constants

rho_w = 1.03e3;         % Density of saltwater
d_prop = .4572;         % Propeller diameter:18" (m)
mass = 550;             % Boat mass (kg)
wheelbase = 3.27;       % Distance from mid-keel to mid-rudder (m)
m_battery = 108.86;     % Weight of ballast batteries (kg)
d_battery = .3048;      % Distance of batteries from rotation axis (m)
disp_vessel = 612.35;   % Vessel displacement:1350 lbs (kg)
max_batt_angle = 1.309; % Maximum angle for the battery tray: 75 degrees(radians)
max_sea_state = 12;     % The maxmimum value of the Beaufort Scale
rudder_angle_max = 1;   % Maximum angle of the rudder (radians)
throttle_max = 1;       % Maximum throttle value (% of maximum amps)

% Experimental constants

K_t = 0.5;              % Thrust coefficient
C_d_sea_surge = 340;    % Coefficient of drag for forward motion through water
cg_x = .0508;           % Center of gravity relative to boat center: 1/6' (m)
cg_z = .0508;           % Center of gravity relative to boat center: 1/6' (m)
% Boat center is at 6.5' by the waterline by the axis of symmetry
rudder_angle_dot_max = 10;   % Maximum rate of change of rudder angle (radians/s)
throttle_dot_max = 1;   % Maximum rate of change of throttle (%/s)
proportional_band = .5;      % Proportional band of the rudder response (radians)
roll_period = 3;        % Periodicity of roll. (s)
power_coefficients = [5.6667 9.6667 0]; % Power coefficients for use in the motor current draw calculations

% L2+ constants
TStar = single(4.5);
IPStar = 0;
InitialPoint = 0;
Turn2Track = 0;
MaxDwnPthStar = 1;
tanIntercept = tan( 45*pi/180 );
ISA_g     = 9.815;          % Gravity               (m/s/s)