#include <xc.h>
#include <stdint.h>
#include "Timer4.h"

// Store the timer callback.
static void (*timer4Callback)(void);

/**
 * Initializes Timer 4 for a 156,250 Hz clock. The prescalar can be used to modify this clock rate
 * to the desied ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being 156250/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer4Init(void (*timerCallbackFcn)(void), uint16_t prescalar)
{
    T4CON = 0;
    IFS1bits.T4IF = 0;
    IPC6bits.T4IP2 = 1;
    IEC1bits.T4IE = 1;
    PR4 = prescalar;
    T4CON = 0x8030;

    timer4Callback = timerCallbackFcn;
}

/**
 * Timer 3 interrupt routine. Merely calls the provided callback function.
 */
void __attribute__((interrupt, auto_psv)) _T4Interrupt(void)
{
	timer4Callback();

    IFS1bits.T4IF = 0;
}