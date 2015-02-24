%% Plot each autonomous run from the provided CSV file.
function static_plots(csv_file)

    data = ProcessCsvFile(csv_file);

    if isfield(data, 'CONTROLLER_DATA')
        static_plots_cdata(data);
    else
        static_plots_gcs(data);
    end

end
