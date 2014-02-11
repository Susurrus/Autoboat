#ifndef RC_CAPTURE_H
#define RC_CAPTURE_H

#include <stdint.h>

/**
 * Initializes IC1, 2, 7, and 8 along with Timer2 for capturing uptime for an input PWM signal.
 * This function DOES NOT configure the Peripheral Pin Select bits.
 * Additionally it assumes IC1 is mapped to RB9, IC2 to RB15, IC7 to RB10, IC8 to RB14.
 * Resultant data can be obtained from the RcCaptureGetX() functions.
 */
void RcCaptureInit(void);

/**
 * Returns the last value obtained from the input capture. Units are in F_OSC/2/256.
 */
uint16_t RcCaptureGet1(void);
uint16_t RcCaptureGet2(void);
uint16_t RcCaptureGet7(void);
uint16_t RcCaptureGet8(void);

#endif // RC_CAPTURE_H