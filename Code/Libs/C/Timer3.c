#include <xc.h>
#include <stdint.h>
#include "Timer3.h"

// Store the timer callback.
static void (*timer3Callback)(void);

/**
 * Initializes Timer 3 for a 156,250 Hz clock. The prescalar can be used to modify this clock rate
 * to the desied ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being 156250/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer3Init(void (*timerCallbackFcn)(void), uint16_t prescalar)
{
    T3CON = 0;
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP2 = 1;
    IEC0bits.T3IE = 1;
    PR3 = prescalar;
    T3CON = 0x8030;

    timer3Callback = timerCallbackFcn;
}

/**
 * Timer 3 interrupt routine. Merely calls the provided callback function.
 */
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{
	timer3Callback();

    IFS0bits.T3IF = 0;
}