PI_2 = pi/2;
PI_4 = pi/4;

% Easy null-test
gyro_rates = [0; 0; 1]; % Body-frame yaw-rate of 1rad/s
attitude = [0 0 0]; % Stay pointing north and that's it
exp_result = gyro_rates; % So we should get the same value
act_result = body_rates_to_euler_rates(attitude, gyro_rates);
assert(isequal(exp_result, act_result));

% Easy null-test
gyro_rates = [0; 1; 0]; % Body-frame pitch-rate of 1rad/s
attitude = [0 0 0]; % Stay pointing north and that's it
exp_result = gyro_rates; % So we should get the same value
act_result = body_rates_to_euler_rates(attitude, gyro_rates);
assert(isequal(exp_result, act_result));

% Easy null-test
gyro_rates = [1; 0; 0]; % Body-frame roll-rate of 1rad/s
attitude = [0 0 0]; % Stay pointing north and that's it
exp_result = gyro_rates; % So we should get the same value
act_result = body_rates_to_euler_rates(attitude, gyro_rates);
assert(isequal(exp_result, act_result));

% Now test when the vehicle is oriented 45 to port
gyro_rates = [0; 0; 1];
attitude = [0, 0, PI_4];
exp_result = gyro_rates;
act_result = body_rates_to_euler_rates(attitude, gyro_rates);
assert(isequal(exp_result, act_result));

% Now test when the vehicle is pitched 45 degrees up the body-frame yaw
% rate will be evenly split between the inertial roll and yaw axes.
gyro_rates = [0; 0; 1];
attitude = [0, PI_4, 0];
exp_result = gyro_rates;
act_result = body_rates_to_euler_rates(attitude, gyro_rates)
assert(isequal(exp_result, act_result));

% Now only roll the vehicle 45 degrees and test again
gyro_rates = [0; 0; 1];
attitude = [PI_4, 0, 0];
exp_result = gyro_rates;
act_result = body_rates_to_euler_rates(attitude, gyro_rates)
assert(isequal(exp_result, act_result));

% And another one
gyro_rates = [0; 0; 1];
attitude = [0, PI_4, PI_2];
exp_result = [];
act_result = body_rates_to_euler_rates(attitude, gyro_rates)


disp('SUCCESS: All tests passed for body_rates_to_euler_rates()');