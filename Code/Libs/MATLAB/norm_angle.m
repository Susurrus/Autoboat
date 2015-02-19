function [out] = norm_angle(in)
%NORM_ANGLE Normalizes input angles to range [0..2*pi]
%   Input vector is expected to be radian angles between -pi and pi. This
%   function normalizes those into the range 0 and 2*pi.
    values_to_fix = find(in < 0);
    in(values_to_fix) = in(values_to_fix) + 2*pi;
    out = in;
end

