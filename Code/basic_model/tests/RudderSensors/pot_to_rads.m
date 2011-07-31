function [rads] = pot_to_rads(in, port_limit, starboard_limit, reset)

persistent low_side high_side lastInput offset
if isempty(low_side) || isempty(high_side) || reset
    low_side = int16(-1);
    high_side = int16(-1);
    lastInput = int16(-10000);
    offset = int16(0);
    rads = 0;
    return;
end

% Here we track transitions. On a high->low transition start adding 1023
% and on a low->high transition don't add anything. It "centers" the input
% range around 1023.
input = int16(in);
if lastInput == uint16(-10000)
    lastInput = input;
end
jump = input - lastInput;
if jump > 1000
    offset = int16(0);
elseif jump < -1000
    offset = int16(1023);
end

% We record the last input BEFORE it's offset. This fixes situations where
% we're comparing data before and after the very first transition.
lastInput = input;

% Apply any offset that should be there.
input = input + offset;

% Update port range limit when the Hall limit switch. Only update to the
% maximum value obtained during the limit switch active period.
if port_limit && (input > high_side || high_side == -1)
    high_side = input;
    
    % If the high_side limit is less than the low_side one, we add 1023
    % assuming that it's a valid value (ie. ~= -1).
    if high_side < low_side && high_side ~= -1
        high_side = high_side + 1023;
    end
end

% Update starboard ADC value when limit is reached. Only update to the
% maximum value obtained during the limit switch active period.
if starboard_limit && (input < low_side || low_side == -1)
    low_side = input;
    
    % If the high_side limit is less than the low_side one, we add 1023
    % assuming that it's a valid value (ie. ~= -1).
    if high_side < low_side && high_side ~= -1
        high_side = high_side + 1023;
    end
end

% Return 0 when we can't actually determine the rudder position
if high_side == -1 || low_side == -1
    rads = double(0);
    return;
end

% Prepare our input. Here we subtract the baseline 'low_side' value off of
% the input so we start with a range from [0..high_side-lowside] and map it
% into the output range of 45 degrees.
in_min = 0;
in_max = high_side - low_side;
out_min = int16(-45);
out_max = int16(45);
val = input - low_side;

% This is a basic integer implementation of a map() function. It was pulled
% from: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1289758376, first
% post.
% NOTE: There can be integer overflow in the first multiplication.
mappedValue = (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

% Finally convert to radians and a double before returning it.
rads = double(mappedValue) * pi / 180;