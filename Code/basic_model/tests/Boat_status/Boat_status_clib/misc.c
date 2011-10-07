#include <stdio.h>

/**
 * This function returns some nice formatted text using ANSI control codes to
 * render them nicely.
 * Control codes:
 *  - ESC[2J <- clears the screwwn
 *  - ESC[1;1H <- Returns cursor to home position
 */
void formatOutputString(unsigned short rudderPosition, unsigned char* outputString) {
	sprintf(outputString, "%c[2J%c[1;1HTest text:%0+4d", 27, 27, rudderPosition);
}