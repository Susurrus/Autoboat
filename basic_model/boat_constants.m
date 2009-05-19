%% Declare boat constants

% Initial conditions

e_0 = 0;                % Initial east position
n_0 = 0;                % Initial north position
phi_0 = 0;              % Initial heading
sea_state = 0;          % The sea state on the Beaufort Scale
T_step = 0.01;          % Simulation timestep
battery_tray_angle = 0; % Battery tray angle
initial_charge = 20;    % Initial boat charge (J)

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

% Experimental constants

wind_effect = 0.1;      % Wind effect coefficient. What % of the wind velocity affects the boat velocity
current_effect = 0.1;   % Ocean current effect coefficient. What % of the current velocity affects the boat velocity
K_t = 0.5;              % Thrust coefficient
C_d_sea_surge = 340;    % Coefficient of drag for forward motion through water
cg_x = .0508;           % Center of gravity relative to boat center: 1/6' (m)
cg_z = .0508;           % Center of gravity relative to boat center: 1/6' (m)
% Boat center is at 6.5' by the waterline by the axis of symmetry
rudder_angle_dot_max = 10;   % Maximum rate of change of rudder angle (radians/s)
proportional_band = .5;      % Proportional band of the rudder response (radians)
roll_period = 3;        % Periodicity of roll. (s)
power_coefficients = [5.6667 9.6667 0]; % Power coefficients for use in the motor current draw calculations