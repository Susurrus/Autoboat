function att_ypr_dot = body_rates_to_euler_rates(att_ypr, gyro_b)
%% This function rotates gyro data from body-axis to Euler-angle
%#codegen
% See https://www.princeton.edu/~stengel/MAE331Lecture9.pdf and
% http://www.chrobotics.com/library/understanding-euler-angles for the
% derivation.
% att_ypr a 3-vector with the roll-pitch-yaw angle in rads (phi,theta,psi)
% gyro_b the body-axis rates (referred to as p,q,r or omega_x,omega_y,omega_x)
% att_ypr_dot The angular rates in the navigation frame (roll-rate,
%             pitch-rate, yaw-rate)

phi = att_ypr(1); % roll
theta = att_ypr(2); % pitch

sin_phi = sin(phi);
cos_phi = cos(phi);
tan_theta = tan(theta);
sec_theta = sec(theta);

C_b_to_n = [1 sin_phi*tan_theta cos_phi*tan_theta;
            0     cos_phi          -sin_phi;
            0 sin_phi*sec_theta cos_phi*sec_theta];

att_ypr_dot = C_b_to_n * gyro_b;