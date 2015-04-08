function Busses() 
% NEWBUSSES initializes a set of bus objects in the MATLAB base workspace 

% Bus object: MavlinkData 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'LocalPosition';
elems(1).Dimensions = 3;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'Velocity';
elems(2).Dimensions = 3;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Course';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'Speed';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'single';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'AimPoint';
elems(5).Dimensions = 3;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'single';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'L2Vector';
elems(6).Dimensions = 3;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'single';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'Acmd';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'single';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'wp0';
elems(8).Dimensions = 3;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'single';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'wp1';
elems(9).Dimensions = 3;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'single';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'sensedYaw';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'single';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

elems(11) = Simulink.BusElement;
elems(11).Name = 'sensedYawRate';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'single';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];

elems(12) = Simulink.BusElement;
elems(12).Name = 'gpsVelOffset';
elems(12).Dimensions = 3;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'single';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];

elems(13) = Simulink.BusElement;
elems(13).Name = 'gpsPosOffset';
elems(13).Dimensions = 3;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'single';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];

elems(14) = Simulink.BusElement;
elems(14).Name = 'gpsPos';
elems(14).Dimensions = 3;
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'single';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];

elems(15) = Simulink.BusElement;
elems(15).Name = 'wpReachedIndex';
elems(15).Dimensions = 1;
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'int8';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];

elems(16) = Simulink.BusElement;
elems(16).Name = 'wpCurrentIndex';
elems(16).Dimensions = 1;
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'int8';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];

InternalVariables = Simulink.Bus;
InternalVariables.Description = sprintf('Holds all of the data generated within Simulink code that Mavlink may need access to for transmission.');
InternalVariables.DataScope = 'Exported';
InternalVariables.Alignment = -1;
InternalVariables.Elements = elems;
assignin('base', 'InternalVariables', InternalVariables);

% Bus object: Mission 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'coordinates'; % Default coordinates for reference frame
elems(1).Dimensions = 3;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'otherCoordinates'; % Local if the mission is global and vice versa.
elems(2).Dimensions = 3;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'refFrame';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'action';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'parameters';
elems(5).Dimensions = 4;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'single';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'autocontinue';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'boolean';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

Mission = Simulink.Bus;
Mission.Description = sprintf('Contains all of the parameters necessary for fully defining missions (aka waypoints) for the autopilot. Follows directly from how MAVLink implements missions.');
Mission.DataScope = 'Imported';
Mission.HeaderFile = 'Missions.h';
Mission.Alignment = -1;
Mission.Elements = elems;
assignin('base', 'Mission', Mission)

% Bus object: MissionList 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'currentIndex';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'size';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'updated';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'maxSize';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'startingPoint';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Bus: Mission';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'missions';
elems(6).Dimensions = 16;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'Bus: Mission';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

MissionList = Simulink.Bus;
MissionList.Description = sprintf('This is the central data store for the various Missions that will be used by the autopilot.');
MissionList.DataScope = 'Exported';
MissionList.Alignment = -1;
MissionList.Elements = elems;
assignin('base', 'MissionList', MissionList)

% Bus object: GpsData 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'newData';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'mode';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'cog';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint16';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'sog';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'hdop';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int16';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'vdop';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int16';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'latitude';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'int32';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'longitude';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'int32';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'altitude';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'int32';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'variation';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'single';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

GpsData = Simulink.Bus;
GpsData.Description = sprintf('Struct storing all GPS data recorded.');
GpsData.DataScope = 'Imported'; % Specify that the header file declaring this struct is external
GpsData.HeaderFile = 'EcanSensors.h'; % Specify the name of the file declaring this datatype.
GpsData.Alignment = -1;
GpsData.Elements = elems;
assignin('base', 'GpsData', GpsData)

% Bus object: ImuData
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'newData';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'attitude';
elems(2).Dimensions = 3;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'gyros';
elems(3).Dimensions = 3;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'accels';
elems(4).Dimensions = 3;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'single';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

ImuData = Simulink.Bus;
ImuData.Description = sprintf('Struct storing all IMU data recorded.');
ImuData.DataScope = 'Imported'; % Specify that the header file declaring this struct is external
ImuData.HeaderFile = 'EcanSensors.h'; % Specify the name of the file declaring this datatype.
ImuData.Alignment = -1;
ImuData.Elements = elems;
assignin('base', 'ImuData', ImuData)

