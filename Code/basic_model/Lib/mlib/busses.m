function busses() 
% BUSSES initializes a set of bus objects in the MATLAB base workspace 

% Bus object: BallastOutBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'enable';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'direction';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

BallastOutBus = Simulink.Bus;
BallastOutBus.HeaderFile = '';
BallastOutBus.Description = sprintf('');
BallastOutBus.Elements = elems;
assignin('base', 'BallastOutBus', BallastOutBus)

% Bus object: BallastSensorBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'position';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint16';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'port_limit';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

elems(3) = Simulink.BusElement;
elems(3).Name = 'starboard_limit';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';

BallastSensorBus = Simulink.Bus;
BallastSensorBus.HeaderFile = '';
BallastSensorBus.Description = sprintf('');
BallastSensorBus.Elements = elems;
assignin('base', 'BallastSensorBus', BallastSensorBus)

% Bus object: GpsBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'latitude';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'longitude';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

elems(3) = Simulink.BusElement;
elems(3).Name = 'altitude';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';

elems(4) = Simulink.BusElement;
elems(4).Name = 'year';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';

elems(5) = Simulink.BusElement;
elems(5).Name = 'month';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'uint8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';

elems(6) = Simulink.BusElement;
elems(6).Name = 'day';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'uint8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';

elems(7) = Simulink.BusElement;
elems(7).Name = 'hour';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';

elems(8) = Simulink.BusElement;
elems(8).Name = 'minute';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';

elems(9) = Simulink.BusElement;
elems(9).Name = 'second';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'uint8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';

elems(10) = Simulink.BusElement;
elems(10).Name = 'cog';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'single';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';

elems(11) = Simulink.BusElement;
elems(11).Name = 'sog';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'single';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';

elems(12) = Simulink.BusElement;
elems(12).Name = 'hdop';
elems(12).Dimensions = 1;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'single';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';

elems(13) = Simulink.BusElement;
elems(13).Name = 'fix';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'uint8';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';

elems(14) = Simulink.BusElement;
elems(14).Name = 'satellites';
elems(14).Dimensions = 1;
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'uint8';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';

elems(15) = Simulink.BusElement;
elems(15).Name = 'newData';
elems(15).Dimensions = 1;
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'boolean';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';

GpsBus = Simulink.Bus;
GpsBus.HeaderFile = '';
GpsBus.Description = sprintf('');
GpsBus.Elements = elems;
assignin('base', 'GpsBus', GpsBus)

% Bus object: RudderOutBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'enable';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'direction';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

elems(3) = Simulink.BusElement;
elems(3).Name = 'up';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint16';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';

elems(4) = Simulink.BusElement;
elems(4).Name = 'period';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';

RudderOutBus = Simulink.Bus;
RudderOutBus.HeaderFile = '';
RudderOutBus.Description = sprintf('');
RudderOutBus.Elements = elems;
assignin('base', 'RudderOutBus', RudderOutBus)

% Bus object: RudderSensorBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'position';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint16';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'port_limit';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

elems(3) = Simulink.BusElement;
elems(3).Name = 'starboard_limit';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';

RudderSensorBus = Simulink.Bus;
RudderSensorBus.HeaderFile = '';
RudderSensorBus.Description = sprintf('');
RudderSensorBus.Elements = elems;
assignin('base', 'RudderSensorBus', RudderSensorBus)

% Bus object: ThrottleOutBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'identifier';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

elems(2) = Simulink.BusElement;
elems(2).Name = 'data';
elems(2).Dimensions = 6;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';

elems(3) = Simulink.BusElement;
elems(3).Name = 'size';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';

elems(4) = Simulink.BusElement;
elems(4).Name = 'trigger';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'boolean';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';

ThrottleOutBus = Simulink.Bus;
ThrottleOutBus.HeaderFile = '';
ThrottleOutBus.Description = sprintf('');
ThrottleOutBus.Elements = elems;
assignin('base', 'ThrottleOutBus', ThrottleOutBus)

% Bus object: ThrottleSensorBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'speed';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int16';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';

ThrottleSensorBus = Simulink.Bus;
ThrottleSensorBus.HeaderFile = '';
ThrottleSensorBus.Description = sprintf('');
ThrottleSensorBus.Elements = elems;
assignin('base', 'ThrottleSensorBus', ThrottleSensorBus)

