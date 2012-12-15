#ifndef TIMER4_H
#define TIMER4_H

#include <xc.h>

/**
 * Resets the timer4 counter.
 */
#define TIMER4_RESET    TMR4 = 0

/**
 * Enables timer4 clearing its counter as it does so.
 */
#define TIMER4_ENABLE   T4CONbits.TON = 1

/**
 * Disables Timer4.
 */
#define TIMER4_DISABLE  T4CONbits.TON = 0

/**
 * Initializes Timer 3 to use the CPU clock. The prescalar can be used to modify this clock rate
 * to the desired ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being FREQ/2/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer4Init(void (*timerCallbackFcn)(void), uint16_t prescalar);

#endif // TIMER4_H