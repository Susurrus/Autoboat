#include <stdio.h>
#include <p33fxxxx.h>
#include <string.h>

/**
 * Generates a current control command string. Command is only generated if num range is valid.
 * This has the form CC#### where #### is a number between and including -1024 and 1023.
 */
extern void send_cc_command(short num, unsigned char enable) {
  // Enable or disable UART transmission depending on the enable line.
  if (enable == 1) {
    U2STAbits.UTXEN = 1;
  }
  else {
    U2STAbits.UTXEN = 0;
  }
  // Make sure we limit our integer value. This ensures we generate a proper command.
  if (num <= 1023 && num >= -1024 && enable) {
    char ss[8];
    sprintf(ss, "%.4d\r\n", num); // Convert our number into ASCII from base10
    char i = 0;
    while (i < 8) {
      while(U2STAbits.UTXBF==1); // Wait for buffer to be ready for more data
      U2TXREG = ss[i];
      ++i;
    }
  }
}

/**
 * Generates a run switch command string. Command is only generated if num range is valid.
 * This has the form SR# where # is a number from 0 to 3.
 * 0 -> Drive in standby
 * 1 -> Drive in run mode if enabled
 * 2 -> Forced standby
 * 3 -> Forced run
 */
extern void send_sr_command(char num, char* ss) {
  // Make sure we limit our integer value. This ensures we generate a proper command.
  if (num <= 3 && num >= 0) {
    sprintf(ss, "SR%.1d\n", num); // Convert our number into ASCII from base10
  }
}
