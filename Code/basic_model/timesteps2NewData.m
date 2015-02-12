function [out_values, out_time] = timesteps2NewData(timestamps, stepsize)
% This function evaluates the given input timestamps as if they indicate
% the true values in a real data set. It's assumed that if a value is true,
% true should be the value at the next timestep, but then if no new value
% is seen, it will be false by the next timestep. This function is used for
% converting timesteps into a boolean newData signal at the specified
% timestep.
    in_time = timestamps;
    
    % Determine the desired output timestamps
    out_time = (0:stepsize:in_time(end))';

    % The general procedure is to merge the input timestamps and the desired
    % timestamps using the value 3 for our inserted timestamps.
    all_times = [out_time; in_time]; % output times must be before input-times so that things work nicely with unique
    out_time_indices = logical([ones(size(out_time)); zeros(size(in_time));]); % Indices of the out_time values in the data array
    values = [3*ones(size(out_time)); ones(size(in_time));];                   % The data array

    % Then we sort the time array, mixing the desired timestamps with the original ones.
    [sorted_t, sorted_i] = sort(all_times);                % The unique sorted time array
    sorted_values = values(sorted_i);                      % The unique sorted data array
    sorted_out_time_indices = out_time_indices(sorted_i);  % The out_time indices (map from the time/data array to the output array)

    % And now we find duplicates, making sure the duplicated values are
    % switched to being output so we get the original data.
    sorted_t_d = [diff(sorted_t); 1] == 0;
    sorted_t_dd = [0; diff(sorted_t_d);];
    flip_indices = sorted_t_d - sorted_t_dd;
    sorted_out_time_indices = xor(sorted_out_time_indices, flip_indices);

    % And we make negative all duplicated values so that we can do the right
    % thing later.
    sorted_values = sorted_values + (2 * sorted_t_dd);

    % Now make sure all of the arrays are unique
    [~, uniq_sorted_i] = unique(sorted_t);                           % The unique sorted time array
    uniq_sorted_values = sorted_values(uniq_sorted_i);                      % The unique sorted data array
    uniq_sorted_out_time_indices = sorted_out_time_indices(uniq_sorted_i);  % The out_time indices (map from the time/data array to the output array)

    % This first round is the zero-order hold from the original timesteps onto
    % the new timesteps.
    d_values = [0; diff(uniq_sorted_values)];
    uniq_sorted_values(d_values == 2) = 1; % Going from a 1 to a 3 is just a resampling of the 1.
    uniq_sorted_values(d_values == 4) = 0; % This indicates the previous data point was at the right timestep, so the point that follows it should be a 0

    % Now this next one is making everything else a 0.
    uniq_sorted_values(uniq_sorted_values == 3) = 0; % Going from a 0 to a 3 means the 3 should be a 0

    % Now fix all of the duplicated values that were made negative
    uniq_sorted_values = abs(uniq_sorted_values);
    
    % And finally save just the desired values
    out_values = uniq_sorted_values(uniq_sorted_out_time_indices);

end