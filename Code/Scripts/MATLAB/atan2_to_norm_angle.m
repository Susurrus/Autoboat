function [ out ] = atan2_to_norm_angle( in )
%ATAN2_TO_NORM_ANGLE Maps -pi..pi output of atan2 into 0..2pi range.
%   The output range is from 0..2*pi, but 0 is at the top (0,1).
    if any(find(in < -pi)) || any(find(in > pi))
        error('Values out of range.');
    end
    out = pi/2 - in;
    % Map top-left quadrant
    values_to_fix_ul = find(out < 0);
    out(values_to_fix_ul) = 5*pi/2 - in(values_to_fix_ul);
end

