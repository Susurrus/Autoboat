% This function parses a CSV file into a nice struct that it then returns.
% It handles lines of comments starting with '%' by ignoring them. It
% expects the first line after all the comment lines to be a
% comma-delimited list of column headers. They can be written in the form
% of X.Y and the resulting struct will then be organized as logData.X.Y =
% ACTUAL_DATA.
function logData = processCsvFile(filename)

    fid = fopen(filename);

    % Read the lines until we reach one that doesn't start with a comment
    firstLine = fgetl(fid);
    while firstLine(1) == '%'
        firstLine = fgetl(fid);
    end

    % Assume the first line after comments are header names
    colHeaders = regexp(firstLine, ',[ ]?', 'split');
    clear firstLine;

    % Create a new struct for the data
    logData = struct;

    % Now add all of column headers to denote their data.
    C = textscan(fid, repmat('%f ', 1, length(colHeaders)), 'delimiter', ',');

    % Add each column to the logData struct, building sub-structs from subnames
    % so that variables like L2.x creates logData.L2.x.
    for i = 1:length(colHeaders)
        mainFirstColName = regexp(colHeaders(i), '\.', 'split');
        assert(length(mainFirstColName{1}) == 1 || length(mainFirstColName{1}) == 2, 'Only 2-level structs are supported');
        if length(mainFirstColName{1}) == 2
            logData.(mainFirstColName{1}{1}).(mainFirstColName{1}{2}) = C{i};
        elseif length(mainFirstColName{1}) == 1
            logData.(mainFirstColName{1}{1}) = C{i};
        end
    end

    fclose(fid);