#include <stdio.h>
#include <p33fxxxx.h>
#include <string.h>

/**
 * Uses UART2 for sending integer data as strings. On the Explorer16 board, UART2 is connected to the RS232 hardware.
 * An enable line allows for output to be disabled.
 *
 * To use this command, add the C-function block and add this source file to your list of additional sources by going
 * to Simulation->Configuration Parameters then to Real Time Workshop->Custom Code and add the name of this file
 * to the list of additional source files.
 */
extern void send_short(short num, unsigned char enable) {
  // Enable or disable UART transmission depending on the enable line.
  if (enable == 1) {
    U2STAbits.UTXEN = 1;
  }
  else {
    U2STAbits.UTXEN = 0;
  }
  // Make sure we limit our integer value. This ensures we generate a proper command.
  if (enable) {
    char ss[9];
    sprintf(ss, "%d\r\n", num); // Convert our number into ASCII from base10
    char i = 0;
    while (i < 9) {
      while(U2STAbits.UTXBF==1); // Wait for buffer to be ready for more data
      U2TXREG = ss[i];
      ++i;
    }
  }
}