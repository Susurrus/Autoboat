# Compilation

Compilation is done in two stages: first by MATLAB/Simulink and finally by MPLAB X. The first stage involves code generation (see the project README for software requirements). The second stage with MPLAB X involves compiling the resultant MPLAB X project and manually setting the Memory Model to 'large'.

This project requires both the '/Code/Libs/MATLAB' folder and the MicroSimulink-Library project be added to MATLAB's path to compile.
