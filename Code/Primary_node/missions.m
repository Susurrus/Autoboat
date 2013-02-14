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

% Create some test coordinates in the global frame. Useful for testing.
test_coordinates = [36.9525669    -122.0102978    0;
                    36.957368     -122.0102119    1;
                    36.9577109    -121.9973373    2;
                    36.9520867    -121.9976807    3;
                    36.9488620    -122.0033455    4;
                    36.9590826    -122.0042896    5;
                    ];
% The following line just initializes an array of structs
test_waypoints(size(test_coordinates,1)) = Simulink.Bus.createMATLABStruct('Mission');
for i = 1:size(test_coordinates,1)
    m = Simulink.Bus.createMATLABStruct('Mission');
    m.coordinates = single(test_coordinates(i,:));
    m.refFrame = uint8(0); % Set Global
    m.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
    m.autocontinue = true;
    test_waypoints(i) = m;
end

test_coordinates_local = [0    0    0;
                    25   -36  0;
                    145  -44  0;
                    220  -37  0;
                    253  -57  0;
                    312  -56  0;
                   ];
% The following line just initializes an array of structs
test_waypoints_local(size(test_coordinates_local,1)) = Simulink.Bus.createMATLABStruct('Mission');
for i = 1:size(test_coordinates_local,1)
    m = Simulink.Bus.createMATLABStruct('Mission');
    m.coordinates = single(test_coordinates_local(i,:));
    m.refFrame = uint8(1); % Set Local-NED
    m.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
    m.autocontinue = true;
    test_waypoints_local(i) = m;
end

figure8_coordinates = [0    0    0;
                       -35  19   0;
                       -93  21   0;
                       -156 13   0;
                       -186 33   0;
                       -244 34   0;
                       -249 77   0;
                      ];
% The following line just initializes an array of structs
figure8_waypoints(size(figure8_coordinates,1)) = Simulink.Bus.createMATLABStruct('Mission');
for i = 1:size(figure8_coordinates,1)
    m = Simulink.Bus.createMATLABStruct('Mission');
    m.coordinates = single(figure8_coordinates(i,:));
    m.refFrame = uint8(1); % Set Local-NED
    m.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
    m.autocontinue = true;
    figure8_waypoints(i) = m;
end

sampling_coordinates = [0    0    0;
                        2    -45  0;
                       ];
% The following line just initializes an array of structs
sampling_waypoints(size(sampling_coordinates,1)) = Simulink.Bus.createMATLABStruct('Mission');
for i = 1:size(sampling_coordinates,1)
    m = Simulink.Bus.createMATLABStruct('Mission');
    m.coordinates = single(sampling_coordinates(i,:));
    m.refFrame = uint8(1); % Set Local-NED
    m.action = uint8(16); % MAV_CMD_NAV_WAYPOINT
    m.autocontinue = true;
    sampling_waypoints(i) = m;
end

