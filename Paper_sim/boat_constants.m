%% Declare boat constants

% Initial conditions

e_0 = 0;                % Initial east position
n_0 = 0;                % Initial north position
psi_0 = 0;              % Initial heading
T_step = 0.01;          % Simulation step time

% Known constants

rho_w = 1.03e3;         % Density of saltwater
d_prop = .4572;         % Propeller diameter:18" (m)
mass = 550;             % Boat mass (kg)
wheelbase = 3.27;       % Distance from mid-keel to mid-rudder (m)
m_battery = 108.86;     % Weight of ballast batteries (kg)
d_battery = .3048;      % Distance of batteries from rotation axis (m)
disp_vessel = 612.35;   % Vessel displacement:1350 lbs (kg)
max_batt_angle = 1.309; % Maximum angle for the battery tray: 75 degrees(radians)
rudder_angle_max = 1;   % Maximum angle of the rudder (radians)

% Experimental constants
% These are approximated constants. Need to be experimentally determined and then they will be known.

K_t = 0.5;              % Thrust coefficient
C_d_sea_surge = 340;    % Coefficient of drag for forward motion through water
cg_x = .0508;           % Center of gravity relative to boat center: 1/6' (m)
cg_z = .0508;           % Center of gravity relative to boat center: 1/6' (m)
% Boat center is at 6.5' by the waterline by the axis of symmetry
rudder_angle_dot_max = 10;   % Maximum rate of change of rudder angle (radians/s)
proportional_band = .5;      % Proportional band of the rudder response (radians)
rudder_stall_angle = .279;   % Stall angle for the rudder:16 degrees[from http://www.boatdesign.net/forums/sailboats/rudder-angle-effective-size-rudder-10724.html] (radians)
rudder_area = .0945;    % Area of the rudder [see notes for calculation] (square meters)
rudder_lift_coefficient = 1.15; % Coefficient of lift for the rudder
rudder_span = .4572;    % Distance to rudder along rotation axis (meters)
cp_x = 3.4224;          % Distance from fixed-body origin along x-axis of center of pressure of the rudder (m)
cp_z = .2032;           % Distance from fixed-body origin along z-axis of center of pressure of the rudder (m)

% Moments of Inertia
% Approximated using l=13,w=3,h=4 for a solid cuboid
I_xx = mass/12*(3^2+4^2);
I_yy = mass/12*(13^2+4^2);
I_zz = mass/12*(13^2+3^2);