%% This file compiles the simulation file for use on a dsPIC
%% microcontroller.

% This compilation flow follows the following steps:
%  1) Copy the controller from Boat_sim.mdl into existing codegen.mdl
%  replacing the dummy block there.
%  2) Builds the model with the RTW Embedded Coder to generate a .hex file
load_system('boat_sim');
load_system('code_gen');
replace_block('code_gen', 'Name', 'Autonomous Controller', 'boat_sim/Autonomous Controller', 'noprompt');
close_system('code_gen', 1); % Close code_gen without saving
close_system('boat_sim');
rtwbuild('code_gen');