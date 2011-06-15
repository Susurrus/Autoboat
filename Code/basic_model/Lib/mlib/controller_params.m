OC2max = 49999; % Parameter used for calculating up-time & period for output compare 2.

C_d_sea_surge = 340;    % Coefficient of drag for forward motion through water
rho_w = 1.03e3;         % Density of saltwater
d_prop = .4572;         % Propeller diameter:18" (m)
K_t = 0.5;              % Thrust coefficient
wheelbase = 3.27;       % Distance from mid-keel to mid-rudder (m)
phi_0 = 0;              % Initial heading (eastward positive from north)
mass = 550;             % Boat mass (kg)

% L2+ constants
TStar = single(4.5);
IPStar = 0;
InitialPoint = 0;
Turn2Track = 0;
MaxDwnPthStar = 1;
tanIntercept = tan( 45*pi/180 );
ISA_g     = 9.815;          % Gravity               (m/s/s)
switchDistance = 4; % Meters before reaching a waypoint that you will then switch over to the next waypoint