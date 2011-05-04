%% Plot PC, GS, and Sim velocity

hold on;
axis equal;

% Keep track of what's on the plot for a legend() call
myLegend = {};

% Plot velocity vectors
decoration_steps = 1:10*pcPace:length(pc.position);
quiver(pc.position(decoration_steps,2), pc.position(decoration_steps,1), pc.velocity(decoration_steps,2), pc.velocity(decoration_steps,1), 'k:');
myLegend{end + 1} = 'PC Velocity';

decoration_steps = 1:10*gsPace:length(gs.localPosition);
quiver(gs.localPosition(decoration_steps,2), gs.localPosition(decoration_steps,1), gs.velocity(decoration_steps,2), gs.velocity(decoration_steps,1), 'b-.');
myLegend{end + 1} = 'GS Velocity';

decoration_steps = 1:10*pcPace:length(pc.sensedPosition);
quiver(pc.sensedPosition(decoration_steps,2), pc.sensedPosition(decoration_steps,1), pc.sensedVelocity(decoration_steps,2), pc.sensedVelocity(decoration_steps,1), 'r--');
myLegend{end + 1} = 'Sim Velocity';

legend(myLegend);

hold off;