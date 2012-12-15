#ifndef TIMER2_H
#define TIMER2_H

#include <xc.h>

/**
 * Resets the timer2 counter.
 */
#define TIMER2_RESET    TMR2 = 0

/**
 * Enables timer2 clearing its counter as it does so.
 */
#define TIMER2_ENABLE   T2CONbits.TON = 1

/**
 * Disables Timer2.
 */
#define TIMER2_DISABLE  T2CONbits.TON = 0

/**
 * Initializes Timer 2 to use the CPU clock. The prescalar can be used to modify this clock rate
 * to the desired ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being FREQ/2/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer2Init(void (*timerCallbackFcn)(void), uint16_t prescalar);

#endif // TIMER2_H