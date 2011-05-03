%% Plot PC and GS velocity

figure(1);clf;
hold on;
axis equal;

% Clean up some variables that may have singleton dimensions:
pc.position = squeeze(pc.position);
pc.velocity = squeeze(pc.velocity);

if size(pc.position, 1) == 3
    pc.position = pc.position';
end
if size(pc.velocity, 1) == 3
    pc.velocity = pc.velocity';
end

% Keep track of what's on the plot for a legend() call
myLegend = {};

% Plot velocity vectors
decoration_steps = 1:10*pcPace:length(pc.position);
plot(pc.position(decoration_steps,2), pc.position(decoration_steps,1), 'r');
myLegend{end + 1} = 'PC Velocity';

decoration_steps = 1:10*gsPace:length(gs.localPosition);
plot(gs.localPosition(decoration_steps,2), gs.localPosition(decoration_steps,1), 'b');
myLegend{end + 1} = 'GS Velocity';

legend(myLegend);

hold off;