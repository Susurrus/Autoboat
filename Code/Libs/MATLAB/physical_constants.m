%% Contains physical real-world constants. Includes both environmental and model constants

% Known constants
% Distance from mid-keel to mid-rudder (m)
wheelbase = Simulink.Parameter;
wheelbase.Description = 'A gain on how quickly the boat turns. Part of the inverted bicycle model.';
wheelbase.Value = 3.27;
wheelbase.DataType = 'single';
wheelbase.DocUnits = 'm';
wheelbase.RTWInfo.StorageClass = 'ExportedGlobal';
wheelbase.RTWInfo.Alias = 'wheelbase';