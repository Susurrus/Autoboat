clear all;

% Obtain the current directory name
[~, cwd, ~] = fileparts(pwd());

fprintf('Processing directory "%s"\n', cwd);

% Store the final output XML in this variable
kmlString = '';

% Get a color ordering for the output plots
colordef(gcf, 'black');
tmpColors = int16(get(gcf, 'DefaultAxesColorOrder')*255);
close(gcf);
colors = cell(length(tmpColors), 1);
for i = 1:length(tmpColors)
    colors{i} = ['ff' sprintf('%02x',tmpColors(i, :))];
end
matFiles = dir('*.mat');
for i = 1:length(matFiles)
    
    name = matFiles(i).name;
    % Load the current .mat file
    tmp = load(name);
    
    % Only grab valid GPS fixes.
    validFixes = tmp.gpsFix > 0;
    
    if isempty(validFixes(validFixes > 0))
        fprintf('No valid GPS fixes found in "%s". Skipping file.\n', name);
        continue;
    end

    % Convert GPS positions to degrees in lat/long
    lat = tmp.globalPosition(validFixes, 1)*180/pi;
    long = tmp.globalPosition(validFixes, 2)*180/pi;

    % Now we remove all consecutive duplicates of lat/long tuples.
    [j, ~] = find([1 1 1;diff(tmp.globalPosition(validFixes,:))] ~= 0);
    j = unique(j);
    lat = lat(j);
    long = long(j);

    % Let the user know how long the track is.
    fprintf('Plotting %d points from %s.\n', length(j), name);

    % Get the timestamps for all of these positions
    dateVector = [2000+double(tmp.gpsYear(j)) double(tmp.gpsMonth(j)) double(tmp.gpsDay(j)) double(tmp.gpsHour(j)) double(tmp.gpsMinute(j)) double(tmp.gpsSecond(j))];

    % Be sure to tack on another timestamp at the end so that we can declare a
    % stop time for the last GPS position.
    dateVector(end + 1, :) = dateVector(end, :);
    dateVector(end, 6) = dateVector(end, 6) + 1;
    

    % Generate the appropriate timestamps in Google's date string format
    S = 'yyyy-mm-ddTHH:MM:SSZ';
    timestamps = datestr(dateVector, S);

    plotColorIndex = mod(i, length(colors));
    if plotColorIndex == 0
        plotColorIndex = length(colors);
    end
    plotColor = colors{plotColorIndex};
    
    % Create a KML plot of all of the data points
    waypointPlot = ge_plot(long, ...
                           lat, ...
                           'name', name, ...
                           'lineWidth', 1.0, ...
                           'lineColor', plotColor, ...
                           'timeSpanStart', timestamps(1,:), ...
                           'timeSpanStop', timestamps(end-1,:));
    
    % Now store the output from this file
    kmlString = [kmlString waypointPlot];
end
    
% Write the output
ge_output(sprintf('%s.kml', cwd), kmlString);