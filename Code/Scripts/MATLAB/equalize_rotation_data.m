% This function formats rotation data (between 0 and 2pi radians) such that
% it can be easily compared when plotted. So data that used to have
% transitions at 2*pi and 0, no longer have them. Basically the data is
% instead centered around either 2*pi or 0 depending on what the first
% transition is. This program therefore makes the assuming that the data
% only ever crosses one of these transition lines and it hasn't been
% tested in any other situation.
function out_data = equalize_rotation_data(in_data)

    % Gather all of the large displacements in values which should
    % correspond with wrapping around
    up_transitions = find(diff(in_data) > 1);
    down_transitions = find(diff(in_data) < -1);
    
    % Add an endpoint to the shorter array
    if length(up_transitions) > length(down_transitions)
        down_transitions(end+1) = length(in_data);
    elseif length(down_transitions) > length(up_transitions)
        up_transitions(end+1) = length(in_data);
    end
    
    % Determine which direction to equalize the data around.
    offset = -1;
    if up_transitions(1) < down_transitions(1)
        offset = 1;
    end
    
    % First copy over the data. Since only certain intervals need altering,
    % this shouldn't cost too much to do.
    out_data = in_data;
    
    % Now go through every interval and offset it the right amount so that
    % we see a smooth graph.
    for i = 1:max(length(up_transitions), length(down_transitions))
        if offset == 1
            out_data(up_transitions(i)+1:down_transitions(i)) = out_data(up_transitions(i)+1:down_transitions(i)) - 2*pi;
        elseif offset == -1
            out_data(down_transitions(i)+1:up_transitions(i)) = out_data(down_transitions(i)+1:up_transitions(i)) + 2*pi;
        end
    end
    
    
    