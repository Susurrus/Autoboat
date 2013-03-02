% This script breaks the output from the TX Output Multiplexed MATLAB block
% from Lubin's PIC blockset into separate blocks for each of the different
% waveforms. This can make it easier to see what is actually happening.

plots = size(R,2);

figure;
for i=1:plots
    subplot(plots, 1, i);
    title(sprintf('Channel %d', i-1));
    plot(R(i:plots:end, i));
end