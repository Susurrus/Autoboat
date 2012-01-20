%% Declare boat constants

% First import all necessary Bus objects
Busses;

% Initial conditions
rudderCommandIC = struct(...
    'enable', true,...
    'direction', true,...
    'up', uint16(0),...
    'period', uint16(0)...
);
throttleCommandIC = struct(...
    'identifier', true,...
    'data', uint8([0 0 0 0 0 0]),...
    'size', uint8(0),...
    'trigger', false...
);

% Initial latitude & longitude. This provides the baseline for the plant
% to generate GPS data.
initial_LL = [36.80611 -121.79639];
phi_0 = 0;              % Initial heading (radians, eastward positive from north)
v_0 = 0;                % Initial speed (m/s)
battery_tray_angle = 0; % Initial battery tray angle
T_step = 0.01;          % Simulation timestep
telemetryDataRate = 3*T_step;    % Set the transmission rate for the telemetry sent via UART1

% Physical input/output parameters

OC2max = 49999; % Parameter used for calculating up-time & period for output compare 2.

% Waypoint stuff
% Waypoints are all defined within a local tangent plane in meters North,
% East, Down. Three waypoint tracks have been defined below in the
% test_waypoints, figure8_waypoints, and sampling_waypoints matrices. The
% commented-out matrices are the original test ones. Overrides are
% implemented below for the in-harbor boat test.

m1 = Simulink.Bus.createMATLABStruct('Mission');
m1.coordinates = single([0 0 0]);
m1.refFrame = uint8(1); % Set Local-NED
m1.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m1.autocontinue = true;

m2 = Simulink.Bus.createMATLABStruct('Mission');
m2.coordinates = single([25 -36 0]);
m2.refFrame = uint8(1); % Set Local-NED
m2.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m2.autocontinue = true;

m3 = Simulink.Bus.createMATLABStruct('Mission');
m3.coordinates = single([145 -44 0]);
m3.refFrame = uint8(1); % Set Local-NED
m3.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m3.autocontinue = true;

m4 = Simulink.Bus.createMATLABStruct('Mission');
m4.coordinates = single([220 -37 0]);
m4.refFrame = uint8(1); % Set Local-NED
m4.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m4.autocontinue = true;

m5 = Simulink.Bus.createMATLABStruct('Mission');
m5.coordinates = single([253 -57 0]);
m5.refFrame = uint8(1); % Set Local-NED
m5.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m5.autocontinue = true;

m6 = Simulink.Bus.createMATLABStruct('Mission');
m6.coordinates = single([312 -56 0]);
m6.refFrame = uint8(1); % Set Local-NED
m6.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
m6.autocontinue = true;

test_waypoints = [m1 m2 m3 m4 m5 m6];

figure8_waypoints = [m1 m2 m3 m4 m5 m6];

sampling_waypoints = [m1 m2 m3 m4 m5 m6];


% test_waypoints = int32([0    0    0;
%                         25   -36  0;
%                         145  -44  0;
%                         220  -37  0;
%                         253  -57  0;
%                         312  -56  0;
%                         -1   -1   -1;
%                         -1   -1   -1;
%                         -1   -1   -1;
%                         -1   -1   -1;
%                         -1   -1   -1;
%                        ]);
% 
% figure8_waypoints = int32([0    0    0;
%                            -35  19   0;
%                            -93  21   0;
%                            -156 13   0;
%                            -186 33   0;
%                            -244 34   0;
%                            -249 77   0;
%                            -1   -1   -1;
%                            -1   -1   -1;
%                            -1   -1   -1;
%                            -1   -1   -1;
%                           ]);
% 
% sampling_waypoints = int32([0    0    0;
%                             2    -45  0;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                             -1   -1   -1;
%                            ]);

% Original waypoints
% test_waypoints = int32([
%              0   0   0;
%              450 100 0;
%              600 600 0;
%              150 300 0;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%             ]);
% % Figure eight
% figure8_waypoints = int32([
%              0   0   0;
%              30  60  0;
%              0   90  0;
%              -30 60  0;
%              0   0   0;
%              30  -60 0;
%              0   -90 0;
%              -30 -60 0;
%              -1  -1  -1;
%              -1  -1  -1;
%              -1  -1  -1;
%             ]);
% % Sampling pattern
% sampling_waypoints = int32([
%              0    0    0;
%              0    210  0;
%              -30  210  0;
%              -30  0    0;
%              -60  0    0;
%              -60  210  0;
%              -90  210  0;
%              -90  0    0;
%              -120 0    0;
%              -120 210  0;
%              -1   -1   -1;
%             ]);

% Known constants
wheelbase = 3.27;       % Distance from mid-keel to mid-rudder (m)

% L2+ constants
TStar = single(4.5);
IPStar = 0;
InitialPoint = 0;
Turn2Track = 0;
MaxDwnPthStar = 1;
tanIntercept = tan( 45*pi/180 );
ISA_g     = 9.815;          % Gravity               (m/s/s)
switchDistance = 4; % Distance before reaching a waypoint that you will then switch over to the next waypoint (m)