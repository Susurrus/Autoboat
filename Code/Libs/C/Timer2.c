#include <xc.h>
#include <stdint.h>

// Store the timer callback used by timer2.
static void (*timer2Callback)(void);

/**
 * Initializes Timer 2 for a 156,250 Hz clock. The prescalar can be used to modify this clock rate
 * to the desied ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being 156250/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer2Init(void (*timerCallbackFcn)(void), uint16_t prescalar)
{
    T2CON = 0;
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP2 = 1;
    IEC0bits.T2IE = 1;
    PR2 = prescalar;
    T2CON = 0x8030;

    timer2Callback = timerCallbackFcn;
}

/**
 * Timer 2 interrupt. Merely calls the timerCallback() specified in Timer2Init().
 */
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    timer2Callback();

    IFS0bits.T2IF = 0;
}