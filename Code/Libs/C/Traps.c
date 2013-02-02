/**
 * This file defines a set of hard trap routines to handle situations
 * where the processor cannot recover.
 *
 * Every trap has been defined to turn on port A3 and light it up.
 * This corresponds to the red LED on the CAN node boards.
 */

#include <xc.h>

#include "Node.h"

void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void)
{
	INTCON1bits.OSCFAIL = 0;
	FATAL_ERROR();
}

void __attribute__((interrupt, no_auto_psv)) _AddressError(void)
{
	INTCON1bits.ADDRERR = 0;
	FATAL_ERROR();
}
void __attribute__((interrupt, no_auto_psv)) _StackError(void)
{
	INTCON1bits.STKERR = 0;
	FATAL_ERROR();
}

void __attribute__((interrupt, no_auto_psv)) _MathError(void)
{
	INTCON1bits.MATHERR = 0;
	FATAL_ERROR();
}

void __attribute__((interrupt, no_auto_psv)) _DMACError(void)
{
	INTCON1bits.DMACERR = 0;
	FATAL_ERROR();
}