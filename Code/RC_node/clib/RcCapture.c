#include "RcCapture.h"

#include <stdlib.h>

#include <incap.h>
#include <timer.h>

// Store the latest values from the Input Captures. These are in IC units of .0000064s
// These are private values only accessible from the RcCaptureGetX() functions.
static uint16_t upTimeIc1;
static uint16_t upTimeIc2;
static uint16_t upTimeIc7;
static uint16_t upTimeIc8;

// Store intermediate values for the input captures. These are the Timer2 values at the rising edge.
// These are private variables.
static uint16_t riseTimeIc1;
static uint16_t riseTimeIc2;
static uint16_t riseTimeIc7;
static uint16_t riseTimeIc8;

void RcCaptureInit(void)
{
	// Initialize Timer2 to run at F_OSC/2/256 = 156,250Hz. Steps are therefore .0000064s.
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, UINT16_MAX);
	ConfigIntTimer2(T2_INT_OFF);

	// Initialize input capture. We set it to run off timer2 capturing every edge.
	uint16_t icConfig1 = IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_EDGE;
	// IC1 - (analog) Rudder
	OpenCapture1(icConfig1);
	ConfigIntCapture1(IC_INT_ON & IC_INT_PRIOR_7);
	// IC2 - (analog) Throttle
	OpenCapture2(icConfig1);
	ConfigIntCapture2(IC_INT_ON & IC_INT_PRIOR_7);
	// IC7 - (boolean) Mode
	OpenCapture7(icConfig1);
	ConfigIntCapture7(IC_INT_ON & IC_INT_PRIOR_7);
	// IC8 - (boolean) Rudder calibration
	OpenCapture8(icConfig1);
	ConfigIntCapture8(IC_INT_ON & IC_INT_PRIOR_7);
}

uint16_t RcCaptureGet1(void)
{
	return upTimeIc1;
}

uint16_t RcCaptureGet2(void)
{
	return upTimeIc2;
}

uint16_t RcCaptureGet7(void)
{
	return upTimeIc7;
}

uint16_t RcCaptureGet8(void)
{
	return upTimeIc8;
}

void _ISR _IC1Interrupt(void)
{
	// If this pin is now high, this was a rising-edge event and we just log the timer value.
	if (_RB9) {
		riseTimeIc1 = IC1BUF;
	}
	// Otherwise this is a falling-edge event. So now we take the difference from the 2 timesteps
	// for our value, correcting for any timer wrap-around.
	else {
		uint16_t fallTimeIc1 = IC1BUF;
		if (fallTimeIc1 > riseTimeIc1) {
			upTimeIc1 = fallTimeIc1 - riseTimeIc1;
		} else {
			upTimeIc1 = (PR2 - riseTimeIc1) + fallTimeIc1;
		}
	}
	_IC1IF = 0;
}

void _ISR _IC2Interrupt(void)
{
	// If this pin is now high, this was a rising-edge event and we just log the timer value.
	if (_RB15) {
		riseTimeIc2 = IC2BUF;
	}
	// Otherwise this is a falling-edge event. So now we take the difference from the 2 timesteps
	// for our value, correcting for any timer wrap-around.
	else {
		uint16_t fallTimeIc2 = IC2BUF;
		if (fallTimeIc2 > riseTimeIc2) {
			upTimeIc2 = fallTimeIc2 - riseTimeIc2;
		} else {
			upTimeIc2 = (PR2 - riseTimeIc2) + fallTimeIc2;
		}
	}
	_IC2IF = 0;
}

void _ISR _IC7Interrupt(void)
{
	// If this pin is now high, this was a rising-edge event and we just log the timer value.
	if (_RB10) {
		riseTimeIc7 = IC7BUF;
	}
	// Otherwise this is a falling-edge event. So now we take the difference from the 2 timesteps
	// for our value, correcting for any timer wrap-around.
	else {
		uint16_t fallTimeIc7 = IC7BUF;
		if (fallTimeIc7 > riseTimeIc7) {
			upTimeIc7 = fallTimeIc7 - riseTimeIc7;
		} else {
			upTimeIc7 = (PR2 - riseTimeIc7) + fallTimeIc7;
		}
	}
	_IC7IF = 0;
}

void _ISR _IC8Interrupt(void)
{
	// If this pin is now high, this was a rising-edge event and we just log the timer value.
	if (_RB14) {
		riseTimeIc8 = IC8BUF;
	}
	// Otherwise this is a falling-edge event. So now we take the difference from the 2 timesteps
	// for our value, correcting for any timer wrap-around.
	else {
		uint16_t fallTimeIc8 = IC8BUF;
		if (fallTimeIc8 > riseTimeIc8) {
			upTimeIc8 = fallTimeIc8 - riseTimeIc8;
		} else {
			upTimeIc8 = (PR2 - riseTimeIc8) + fallTimeIc8;
		}
	}
	_IC8IF = 0;
}