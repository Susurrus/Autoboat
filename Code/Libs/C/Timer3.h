#ifndef TIMER3_H
#define TIMER3_H

#include <xc.h>

/**
 * Resets the timer3 counter.
 */
#define TIMER3_RESET    TMR3 = 0

/**
 * Enables timer3 clearing its counter as it does so.
 */
#define TIMER3_ENABLE   T3CONbits.TON = 1

/**
 * Disables Timer3.
 */
#define TIMER3_DISABLE  T3CONbits.TON = 0

/**
 * Initializes Timer 3 to use the CPU clock. The prescalar can be used to modify this clock rate
 * to the desired ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being FREQ/2/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer3Init(void (*timerCallbackFcn)(void), uint16_t prescalar);

#endif // TIMER3_H