## Compilation

Compilation is done in two stages: first by MATLAB/Simulink and finally by MPLAB X. The first stage involves code generation (see the project README for software requirements). The second stage with MPLAB X involves compiling the resultant MPLAB X project and manually setting the Memory Model to 'large'.

This project requires both the '/Code/Libs/MATLAB' folder and the MicroSimulink-Library project be added to MATLAB's path to compile.

### MPLAB X Project Settings
 1. Add the following directories to the include search path: 
   * `/Code/Libs/C`
   * `/Code/Libs/MAVLink`
   * `/Code/Libs/MAVLink/seaslug`
   * `/Code/primary_node`
   * `/Code/primary_node/controller_no_lib_ert_rtw`
 2. Specify the large data model
 3. Add all *.c and *.h files under `controller_no_lib_ert_rtw' to the project.
 4. Add all *.c and *.h files in `/Code/primary_node`.
 5. Add missing files as reported by the compiler in `/Code/Libs/C`