%% Plot each autonomous run from the provided CSV file.
function static_plots(csv_file)

    data = ProcessCsvFile(csv_file);
    
    % Check that all timestamps are unique, otherwise things explode later
    assert(length(data.timestamp) == length(unique(data.timestamp)), 'Timestamp data must be unique for calculations');
    
    % And let's start our timestamps at 0, which makes the plots a little
    % nicer.
    data.timestamp = data.timestamp - data.timestamp(1);

    if isfield(data, 'CONTROLLER_DATA')
        static_plots_cdata(data);
    else
        static_plots_gcs(data);
    end

end
