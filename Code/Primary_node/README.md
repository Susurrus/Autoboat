## Compilation

Compilation is done in two stages: first by MATLAB/Simulink and finally by MPLAB X. The first stage involves code generation (see the project README for software requirements). The second stage with MPLAB X involves compiling the resultant MPLAB X project and manually setting the Memory Model to 'large'.

This project requires both the '/Code/Libs/MATLAB' folder and the MicroSimulink-Library project be added to MATLAB's path to compile.

### MPLAB X Project Settings
 1. Add the following directories to the include search path: 
   * `/Code/Libs/C`
   * `/Code/Libs/MAVLink/seaslug`
   * `/Code/primary_node`
   * `/Code/primary_node/controller_ert_rtw`
 2. Specify the large code & large data model
 3. Add "MAVLINK_ALIGNED_FIELDS=0" as a C macro
 4. Add MAVLINK_SEPARATE_HELPERS as a C macro
 4. Add all *.c under `controller_ert_rtw' to the project.
 5. Add all *.c files in `/Code/primary_node`.
 6. Add missing files as reported by the compiler in `/Code/Libs/C`
 7. OPTIONAL: Add traps.c if weird resets occur to see which error is triggering
 
Note, when regenerating MAVLink, there may arise a conflict between the mavlink_helpers.c file we provide and the function prototypes provided in protocol.h. The mavlink_helpers.c file will need to be adjusted by just copying over the functiond definitions from mavlink_helpers.h.

### dsPIC33E versus dsPIC33F

Switching between these processors requires the following changes:
 1. Choose the proper DEES_*.s file based on the processor type selected.

## Functionality

### MAVLink corruption compensation
The UART1 transmission is connected to UART2 reception at the pin-level on the PIC. A separate MAVLink decoding process is run on this and the processor logs when its output stream has become corrupted, resets the UART1 transmission hardware and outputs a STATUSTEXT MAVLink message with the time this occurred.

Additionally, while the data stream is corrupted, pin RB0 is set high, so a low value indicates a valid MAVLink stream.
