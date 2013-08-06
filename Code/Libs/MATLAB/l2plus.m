% Set some constants for use with the L2+ trajectory controller

% The time-offset for the L2+ controller. The lookahead distance is TStar *
% Velocity. Larger values imply lower-gain.
TStar = Simulink.Parameter;
TStar.Description = 'The time-constant for the L2+ controller';
TStar.Value = 2;
TStar.DataType = 'single';
TStar.DocUnits = 's';
TStar.RTWInfo.StorageClass = 'ExportedGlobal';
TStar.RTWInfo.Alias = 'tStar';

% Distance from the first waypoint that the initial point is estimated to
% be. Only matters if InitialPoint is set to True
IPStar = Simulink.Parameter;
IPStar.Description = 'Distance from the 1st waypoint that the initial point should be set to. Only relevant if InitialPoint is True.';
IPStar.Value = 0;
IPStar.DataType = 'single';
IPStar.DocUnits = 'm';
IPStar.RTWInfo.StorageClass = 'ExportedGlobal';
IPStar.RTWInfo.Alias = 'ipStar';

% Boolean indicating whether the initial-point logic will be used.
InitialPoint = Simulink.Parameter;
InitialPoint.Description = 'Indicates whether the initial-point logic will be used';
InitialPoint.Value = 0;
InitialPoint.DataType = 'boolean';
InitialPoint.DocUnits = '';
InitialPoint.RTWInfo.StorageClass = 'ExportedGlobal';
InitialPoint.RTWInfo.Alias = 'initialPoint';

% Boolean indicating whether to turn towards the L2 vector or away from it.
Turn2Track = Simulink.Parameter;
Turn2Track.Description = 'Indicates to turn towards L2 vector versus away from it.';
Turn2Track.Value = 0;
Turn2Track.DataType = 'boolean';
Turn2Track.DocUnits = '';
Turn2Track.RTWInfo.StorageClass = 'ExportedGlobal';
Turn2Track.RTWInfo.Alias = 'turn2Track';

% The maximum distance down the track from a projection of the vehicle's
% current position that should indicate the aim point.
MaxDwnPthStar = Simulink.Parameter;
MaxDwnPthStar.Description = 'Maximum downpath distance of the aimpoint';
MaxDwnPthStar.Value = 1;
MaxDwnPthStar.DataType = 'single';
MaxDwnPthStar.DocUnits = 'm';
MaxDwnPthStar.RTWInfo.StorageClass = 'ExportedGlobal';
MaxDwnPthStar.RTWInfo.Alias = 'maxDwnPthStar';

% The tangent of the maximum intercept angle.
tanIntercept = Simulink.Parameter;
tanIntercept.Description = 'Tangent of the maximum intercept angle';
tanIntercept.Value = tan(45*pi/180);
tanIntercept.DataType = 'single';
tanIntercept.DocUnits = '';
tanIntercept.RTWInfo.StorageClass = 'ExportedGlobal';
tanIntercept.RTWInfo.Alias = 'tanIntercept';

% Distance before reaching a waypoint that you will then switch over to the next waypoint
switchDistance = Simulink.Parameter;
switchDistance.Description = 'Distance from a waypoint where the vehicle is considered at that waypoint.';
switchDistance.Value = 4;
switchDistance.DataType = 'single';
switchDistance.DocUnits = 'm';
switchDistance.RTWInfo.StorageClass = 'ExportedGlobal';
switchDistance.RTWInfo.Alias = 'switchDistance';