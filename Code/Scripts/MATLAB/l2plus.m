% Set some constants for use in the controller. Most of these are
% L2+-related.

% The time-offset for the L2+ controller. The lookahead distance is TStar *
% Velocity. Larger values imply lower-gain.
TStar = Simulink.Parameter;
TStar.Description = 'The time-constant for the L2+ controller';
TStar.Value = 12;
TStar.DataType = 'single';
TStar.DocUnits = 's';
TStar.RTWInfo.StorageClass = 'ExportedGlobal';
TStar.RTWInfo.Alias = 'tStar';

% The maximum distance down the track from a projection of the vehicle's
% current position that should indicate the aim point.
MaxDwnPthStar = Simulink.Parameter;
MaxDwnPthStar.Description = 'Maximum downpath distance of the aimpoint';
MaxDwnPthStar.Value = 2;
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
switchDistance.Value = 10;
switchDistance.DataType = 'single';
switchDistance.DocUnits = 'm';
switchDistance.RTWInfo.StorageClass = 'ExportedGlobal';
switchDistance.RTWInfo.Alias = 'switchDistance';

% Gain on the feedforward heading rate signal
KPsiDot = Simulink.Parameter;
KPsiDot.Description = 'Gain on the heading derivative feedback loop.';
KPsiDot.Value = 0.4;
KPsiDot.DataType = 'single';
KPsiDot.DocUnits = '';
KPsiDot.RTWInfo.StorageClass = 'ExportedGlobal';
KPsiDot.RTWInfo.Alias = 'KPsiDot';

% Toggle the GPS offset correction code
GpsOffsetCorrectionEnable = Simulink.Parameter;
GpsOffsetCorrectionEnable.Description = 'Enables GPS offset correct when enabled.';
GpsOffsetCorrectionEnable.Value = true;
GpsOffsetCorrectionEnable.DataType = 'boolean';
GpsOffsetCorrectionEnable.DocUnits = '';
GpsOffsetCorrectionEnable.RTWInfo.StorageClass = 'ExportedGlobal';
GpsOffsetCorrectionEnable.RTWInfo.Alias = 'GpsOffsetCorrectionEnable';

%% Also throw in some variables here for the PD controller
% The gain on the course error
K_course = Simulink.Parameter;
K_course.Description = 'Course error gain for the PD controller.';
K_course.Value = single(.2);
K_course.DataType = 'single';
K_course.DocUnits = '';
K_course.RTWInfo.StorageClass = 'ExportedGlobal';
K_course.RTWInfo.Alias = 'K_course';

% Gain on the crosstrack error
K_crosstrack = Simulink.Parameter;
K_crosstrack.Description = 'Crosstrack error gain for the PD controller.';
K_crosstrack.Value = single(0.02);
K_crosstrack.DataType = 'single';
K_crosstrack.DocUnits = '';
K_crosstrack.RTWInfo.StorageClass = 'ExportedGlobal';
K_crosstrack.RTWInfo.Alias = 'K_crosstrack';

% Gain on the feedforward heading rate signal
PD_KPsiDot = Simulink.Parameter;
PD_KPsiDot.Description = 'Gain on the heading derivative feedback loop for the PD controller.';
PD_KPsiDot.Value = 1.0;
PD_KPsiDot.DataType = 'single';
PD_KPsiDot.DocUnits = '';
PD_KPsiDot.RTWInfo.StorageClass = 'ExportedGlobal';
PD_KPsiDot.RTWInfo.Alias = 'PD_KPsiDot';