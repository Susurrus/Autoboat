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
elems(3).Name = 'Heading';
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
elems(5).Name = 'L2Vector';
elems(5).Dimensions = 3;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'single';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'Acmd';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'single';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

MavlinkData = Simulink.Bus;
MavlinkData.HeaderFile = '';
MavlinkData.Description = sprintf('Holds all of the data generated within Simulink code that Mavlink may need access to for transmission.');
MavlinkData.DataScope = 'Auto';
MavlinkData.Alignment = -1;
MavlinkData.Elements = elems;
assignin('base', 'MavlinkData', MavlinkData)

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
elems(2).Name = 'otherCoordinates'; % local coordinates if refFrame is global and vice-versa
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
Mission.HeaderFile = '';
Mission.Description = sprintf('Contains all of the parameters necessary for fully defining missions (aka waypoints) for the autopilot. Follows directly from how MAVLink implements missions.');
Mission.DataScope = 'Auto';
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
MissionList.HeaderFile = '';
MissionList.Description = sprintf('This is the central data store for the various Missions that will be used by the autopilot.');
MissionList.DataScope = 'Auto';
MissionList.Alignment = -1;
MissionList.Elements = elems;
assignin('base', 'MissionList', MissionList)
