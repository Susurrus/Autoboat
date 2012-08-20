%% Generate 'proper' rudder data from the test data. rudder_test1 within
%% rudder_test_data.mat is measured data that has been reversed in time to
%% be usable. rudder_test2 and *_test3 are sin() and cos() approximations
%% of realworld data.
%test_rudderPot = rudder_test1;
%test_rudderPot = rudder_test2;
%test_rudderPot = rudder_test3;
%test_rudderPot = (200*cos(0:.01:5) + 1213)';

% Uncomment the following lines to test this code with the limit switch
% starting order reversed.
%test_rudderPot = 1023 - test_rudderPot;

% Uncomment the below lines to test this code across the 1023 boundary
% test_rudderPot = test_rudderPot + 512;
% alterRange = test_rudderPot > 1023;
% test_rudderPot(alterRange) = test_rudderPot(alterRange) - 1023;

%% Plot the data
figure;
subplot(2,1,1);
hold on;

limitRange = 1*uint8(test_rudderPortLimit) + 2*uint8(test_rudderSbLimit);
rudderRange = 1:length(test_rudderPot);
X = [rudderRange';NaN];
Y = [test_rudderPot;NaN];
Z = [limitRange;NaN];
patch(X, Y, Z, 'FaceColor', 'None', 'EdgeColor', 'Flat');
colormap([0 0 0;0 1 0;0 0 1;]);
set(gca, 'XTick', []);
legend('Pot ADC input. Green = port limit');
ylabel('Rudder position (ADC units)');
title('Raw input data');

%% Calculate results
angles = zeros(size(test_rudderPot));
gg = zeros(size(test_rudderPot));
pot_to_rads(0, 0, 0, true);
for i = 1:length(test_rudderPot)
    angles(i) = pot_to_rads(test_rudderPot(i), test_rudderPortLimit(i), test_rudderSbLimit(i), false);
    angles(i) = angles(i) * 180 / pi;
end

%% Plot the results
subplot(2,1,2);

% We 5-sample box-filter the output.
angles = filter([0.2;0.2;0.2;0.2;0.2], 1, angles);
stairs(angles, 'k');
set(gca, 'XTick', []);
legend('5-sample avg output');
ylabel('Rudder angle (deg)');
title('Mapped output');