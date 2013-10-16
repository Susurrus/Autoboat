#include <xc.h>
#include <stdint.h>
#include <timer.h>

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
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, prescalar);
	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);

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