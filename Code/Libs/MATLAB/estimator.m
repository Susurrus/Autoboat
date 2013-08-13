%% Estimator parameters
% This file defines the various parameters used by the position/attitude
% estimators.

% Value indicating how long (in seconds) between samples used for the
% derivative.
cogDotDerivativeDelay = Simulink.Parameter;
cogDotDerivativeDelay.Description = 'Delay between derivative samples for course-over-ground.';
cogDotDerivativeDelay.Value = 0.8;
cogDotDerivativeDelay.DataType = 'single';
cogDotDerivativeDelay.DocUnits = 's';
cogDotDerivativeDelay.RTWInfo.StorageClass = 'ExportedGlobal';
cogDotDerivativeDelay.RTWInfo.Alias = 'cogDotDerivativeDelay';