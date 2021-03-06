function att_dot = body_rates_to_euler_rates(att, gyro_b)
%% This function rotates gyro data from body-axis to Euler-angle
%#codegen
% See https://www.princeton.edu/~stengel/MAE331Lecture9.pdf and
% http://www.chrobotics.com/library/understanding-euler-angles for the
% derivation.
% att     a 3-vector with the roll-pitch-yaw angle in rads (phi,theta,psi)
% gyro_b  the body-axis rates (referred to as p,q,r or omega_x,omega_y,omega_z)
% att_dot The angular rates in the navigation frame (roll-rate,
%         pitch-rate, yaw-rate)

phi = att(1); % roll
theta = att(2); % pitch

sin_phi = sin(phi);
cos_phi = cos(phi);
tan_theta = tan(theta);
sec_theta = sec(theta);

C_b_to_n = [1 sin_phi*tan_theta cos_phi*tan_theta;
            0     cos_phi          -sin_phi;
            0 sin_phi*sec_theta cos_phi*sec_theta];

att_dot = C_b_to_n * gyro_b;