analysis;
figure;
a = subplot(2, 1, 1);
PlotTrack;
b = subplot(2, 1, 2);
PlotVelocity;
linkaxes([a b], 'xy');
