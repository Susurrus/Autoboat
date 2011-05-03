gs = load('groundstation.mat');
pc = load('matlab.mat');

velocity_gs_norm = sum(sqrt(gs.velocity.^2),2);
velocity_pc_norm = sum(sqrt(pc.velocity.^2),2);

figure;
plot(velocity_gs_norm, 'r');
plot(velocity_pc_norm, 'b');