% Load the data in
gs = load('groundstation.mat');
pc = load('matlab.mat');

% Make sure there're no singleton dimensions in our velocity vectors
gs.velocity = squeeze(gs.velocity);
pc.velocity = squeeze(pc.velocity);

% Change them both into row-matrices
if size(pc.velocity, 1) == 3
    pc.velocity = pc.velocity';
end

% Grab the norm of the velocity vectors
velocity_gs_norm = sum(sqrt(gs.velocity.^2),2);
velocity_pc_norm = sum(sqrt(pc.velocity.^2),2);

% Calculat the maximum length of the velocity vectors
max_vel_len = max([max(velocity_gs_norm) max(velocity_pc_norm)]);

% Interpolate the GS velocity data to the same length as the groundstation
% data by relying on the GPS data.

% First we locate the first valid GPS signal in the GS data.
gs_time_sync_hr = find(gs.gpsHour > 0, 1);
gs_time_sync_min = find(gs.gpsMinute > 0, 1);
gs_time_sync_s = find(gs.gpsSecond > 0, 1);
assert(gs_time_sync_hr == gs_time_sync_min == gs_time_sync_s);
gs_time_sync = gs_time_sync_hr;
clear gs_time_sync_*;
disp([gs.gpsYear(gs_time_sync), gs.gpsMonth(gs_time_sync), gs.gpsDay(gs_time_sync), gs.gpsHour(gs_time_sync), gs.gpsMinute(gs_time_sync), gs.gpsSecond(gs_time_sync)]);

% Then we locate the first valid GPS signal in the PC data
pc_time_sync_hr = find(pc.gpsHour > 0, 1);
pc_time_sync_min = find(pc.gpsMinute > 0, 1);
pc_time_sync_s = find(pc.gpsSecond > 0, 1);
assert(pc_time_sync_hr == pc_time_sync_min && pc_time_sync_hr == pc_time_sync_s);
pc_time_sync = pc_time_sync_hr;
clear pc_time_sync_*;
disp([pc.gpsYear(pc_time_sync), pc.gpsMonth(pc_time_sync), pc.gpsDay(pc_time_sync), pc.gpsHour(pc_time_sync), pc.gpsMinute(pc_time_sync), pc.gpsSecond(pc_time_sync)]);

% We take the one with the earlier timestamp and iterate through it until
% the GPS hour and second match. This is done by converting the hh:mm:ss
% timestamp to raw seconds for a comparison
pcTimeStamp = 3600*uint32(pc.gpsHour(pc_time_sync)) + ...
                60*uint32(pc.gpsMinute(pc_time_sync)) + ...
                   uint32(pc.gpsSecond(pc_time_sync));
               
gsTimeStamp = 3600*uint32(gs.gpsHour(gs_time_sync)) + ...
                60*uint32(gs.gpsMinute(gs_time_sync)) + ...
                   uint32(gs.gpsSecond(gs_time_sync));
               
% Then we locate the first time they sync up
assert(pcTimeStamp == gsTimeStamp);
% if pcTimeStamp < gsTimeStamp
%     searchRangeStart = find(pc.gpsHour == gs.gpsHour(gs_time_sync), 1);
%     searchRangeStart = find(pc.gpsMinute(searchRangeStart:end) == gs.gpsMinute(gs_time_sync), 1);
%     searchRangeStart = find(pc.gpsSecond(searchRangeStart:end) == gs.gpsSecond(gs_time_sync), 1);
%     disp([pc.gpsHour(searchRangeStart), pc.gpsMinute(searchRangeStart), pc.gpsSecond(searchRangeStart)]);
% end

% Here we calculate the frequency of data updates per second
gsPace = find(gs.gpsSecond == gs.gpsSecond(gs_time_sync)+2,1) - find(gs.gpsSecond == gs.gpsSecond(gs_time_sync)+1,1);
pcPace = find(pc.gpsSecond == pc.gpsSecond(pc_time_sync)+2,1) - find(pc.gpsSecond == pc.gpsSecond(pc_time_sync)+1,1);

% Plot vectors at each time step for both the GS and PC velocity with a 
% spacing determined by the max_vel_len calculated above.
figure;
quiver(gs.velocity(gs_time_sync:gsPace:end, 1), gs.velocity(gs_time_sync:gsPace:end, 2), 'r');
figure;
plot(velocity_pc_norm, 'b');