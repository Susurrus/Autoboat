%% Set up several mission sets for use with the controller

% First import all necessary Bus objects
Busses;

% Declare an empty mission for us as an initialization value
emptyMission = Simulink.Bus.createMATLABStruct('Mission');

% Waypoints are all defined within a local tangent plane in meters North,
% East, Down. Three waypoint tracks have been defined below in the
% test_waypoints, figure8_waypoints, and sampling_waypoints matrices. The
% commented-out matrices are the original test ones. Overrides are
% implemented below for the in-harbor boat test.

% Basic first test:
basic_test.waypoints = [36.9525669    -122.0102978    0;
                        36.957368     -122.0102119    1;
                        36.9577109    -121.9973373    2;
                        36.9520867    -121.9976807    3;
                        36.9488620    -122.0033455    4;
                        36.9590826    -122.0042896    5;
                       ];
basic_test.refFrame = 0; % Set a global lat/long/alt reference frame

% Basic local coordinate test
basic_test_local.waypoints = [0       0     0;
                              25     -36    0;
                              145    -44    0;
                              220    -37    0;
                              253    -57    0;
                              312    -56    0;
                             ];
basic_test_local.refFrame = 1; % Set a local NED reference frame

% A slalom path through the harbor
harbor_slalom.waypoints = [36.9638319535983655 -122.002068758010864 0;
                           36.9642177152830627 -122.00243353843689  0;
                           36.9646463370846874 -122.002202868461609  0;
                           36.9651821109432746 -122.002519369125366  0;
                           36.9657135948855782 -122.002326250076294  0;
                           36.9661164915306131 -122.002658843994141  0;
                           36.9668236985703729 -122.002594470977783  0;
                           36.9674537502173379 -122.002728581428528  0;
                          ];
harbor_slalom.refFrame = 0; % Set a global lat/long/alt reference frame

% A simple 6-point figure 8
figure8.waypoints = [0    0    0;
                     -35  19   0;
                     -93  21   0;
                     -156 13   0;
                     -186 33   0;
                     -244 34   0;
                     -249 77   0;
                    ];
figure8.refFrame = 1; % Set a local NED reference frame

% Set this variable to the waypoint struct that you'd like to use
test_coordinates = basic_test;

% The following line just initializes an array of structs
test_waypoints(size(test_coordinates.waypoints,1)) = Simulink.Bus.createMATLABStruct('Mission');
for i = 1:size(test_coordinates.waypoints,1)
    m = Simulink.Bus.createMATLABStruct('Mission');
    m.coordinates = single(test_coordinates.waypoints(i,:));
    m.refFrame = uint8(test_coordinates.refFrame); % Set Local-NED
    m.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
    m.autocontinue = true;
    test_waypoints(i) = m;
end

