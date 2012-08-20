lastPass = [];
for i = 1:50:1023
    test_rudderPot = (200*cos(0:.01:5)+i)';

    r_min = min(test_rudderPot);
    r_max = max(test_rudderPot);
    test_rudderPortLimit = test_rudderPot > r_max-0.05*(r_max-r_min);
    test_rudderSbLimit = test_rudderPot < r_min+0.05*(r_max-r_min);
    
    alterRange = test_rudderPot > 1023;
    test_rudderPot(alterRange) = test_rudderPot(alterRange) - 1023;
    alterRange = test_rudderPot < 0;
    test_rudderPot(alterRange) = test_rudderPot(alterRange) + 1023;
    
    rudder_test;
    
    if isempty(lastPass)
        lastPass = angles;
    else
        assert(isempty(lastPass(lastPass ~= angles)));
    end
end