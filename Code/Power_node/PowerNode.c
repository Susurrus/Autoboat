/*
The MIT License

Copyright (c) 2010 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <xc.h>
#include <pps.h>
#include <adc.h>
#include <dma.h>

#include "Ecan1.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"
#include "Types.h"
#include "Node.h"
#include "Timer2.h"
#include "MessageScheduler.h"

// ADC input struct. Provides enough space for 16 inputs (as req'd by the docs). Really only index
// position 1 (temperature sensor) and 5 (voltage sensor) will be populated. But with the scatter-
// gather mode enabled on the ADC, we reserve an array for all 16 possible inputs, so we align to
// 32-byte boundaries instead.
#ifdef __dsPIC33FJ128MC802__
static volatile uint16_t adcDmaBuffer[16] __attribute__((space(dma),aligned(32)));
#elif __dsPIC33EP256MC502__
static volatile uint16_t adcDmaBuffer[16] __attribute__((aligned(32)));
#endif

// Set up the message scheduler for running 3 tasks:
//  * Blinking the status LED at 1Hz
//  * Transmitting the node status at 2Hz
//  * Transmitting power status at 10Hz
#define NUM_TASKS 3
enum {
	TASK_BLINK = 1,
	TASK_TRANSMIT_STATUS,
	TASK_TRANSMIT_POWER
};
uint8_t taskIds[NUM_TASKS] = {
	TASK_BLINK,
	TASK_TRANSMIT_STATUS,
	TASK_TRANSMIT_POWER
};
uint16_t taskTimeSteps[NUM_TASKS][2][8] = {};
uint8_t  taskWeights[NUM_TASKS] = {1, 1, 1}; // All tasks have an equal weighting, as it doesn't matter.
MessageSchedule taskSchedule = {
	NUM_TASKS,
	taskIds,
	taskWeights,
	0,
	taskTimeSteps
};

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

// Define the maximum value of the ADC input
#define ANmax 4095.0f

// Set some function prototypes.
void RunTasks(void);
void Adc1Init(void);

// Set processor configuration settings
#ifdef __dsPIC33FJ128MC802__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 3 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD3);
#elif __dsPIC33EP256MC502__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 2 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD2);
#endif

int main()
{
	/// First step is to move over to the FRC w/ PLL clock from the default FRC clock.
	// Set the clock to 79.84MHz.
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

	// Initiate Clock Switch to FRM oscillator with PLL.
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

	// Wait for Clock switch to occur.
	while (OSCCONbits.COSC != 1);

	// And finally wait for the PLL to lock.
    while (OSCCONbits.LOCK != 1);

	// Initialize status LEDs for use.
	_TRISA3 = 0;
	_TRISA4 = 0;
	_LATA3 = 0;
	_LATA4 = 0;

	// Initialize ADCs for reading voltage and temperature sensors
	Adc1Init();
	
	// Set up some standard node stuff
	nodeId = CAN_NODE_POWER_SENSOR;
	
    // Initialize ECAN1 for input and output using DMA buffers 0 & 2
    Ecan1Init(F_OSC);

	// Set up a timer at 100.0320Hz, where F_timer = F_CY / 256 / prescalar.
	Timer2Init(RunTasks, F_OSC / 2 / 256 / 100);
	
	// Set up all of our tasks.
	// Blink at 1Hz
	if (!AddMessageRepeating(&taskSchedule, TASK_BLINK, 1)) {
		FATAL_ERROR();
	}
	// Transmit node status at 2Hz
	if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_STATUS, 2)) {
		FATAL_ERROR();
	}
	// Transmit power data at 10Hz
	if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_POWER, 10)) {
		FATAL_ERROR();
	}
	
	// And configure the Peripheral Pin Select pins:
	PPSUnLock;

#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(PPS_C1RX, PPS_RP4);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(PPS_C1RX, PPS_RP36);
#endif

	PPSLock;

	// Do nothing because the timer interrupt handles everything.
	while (1);
}

void RunTasks(void)
{
	static uint8_t sequenceID = 0;

	// Do some things at every timestep.
	// Update the voltage reading for this node (stored as dV).
	nodeVoltage = (uint8_t)(adcDmaBuffer[5] * (3.3/ANmax) * (23.0/2.0) * (10.0));
	// Update the voltage reading for this node (stored as degrees C).
	nodeTemp = (int8_t)(adcDmaBuffer[1] * (3.3/ANmax) * (23.0/2.0) * (10.0));

	// Now do some special scheduled things.
	uint8_t tasks[NUM_TASKS];
	uint8_t count = GetMessagesForTimestep(&taskSchedule, tasks);
	int i;
	for (i = 0; i < count; ++i) {
		switch (tasks[i]) {
			case TASK_BLINK:
				_LATA4 ^= 1;
			break;

			case TASK_TRANSMIT_POWER: {
				CanMessage msg;
				float voltage = (float)adcDmaBuffer[0] * (3.3/ANmax) * (1/.06369);
				float amperage = (float)adcDmaBuffer[3] * (3.3/ANmax) * (1/.03660);
				PackagePgn127508(&msg, nodeId, 0, voltage, amperage, NAN, sequenceID++);
				Ecan1Transmit(&msg);
			} break;
			
			case TASK_TRANSMIT_STATUS: {
				NodeTransmitStatus();
			} break;
		}
	}
}

void Adc1Init(void)
{
	// Initialize ADC for reading temperature, 2x power rail voltage, and current draw.
	// Use standard V_ref+/V_ref- voltage references.
	SetChanADC1(
		ADC_CH123_NEG_SAMPLEA_VREFN & ADC_CH123_NEG_SAMPLEB_VREFN & ADC_CH123_POS_SAMPLEA_0_1_2 & ADC_CH123_POS_SAMPLEB_0_1_2,
		ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN0 & ADC_CH0_NEG_SAMPLEB_VREFN
	);
	// Open AN1 (temperature) and AN5 (voltage) pins for 12-bit unsigned integer readings. Also note
	// that this will only store one sample per analog input.
#ifdef __dsPIC33FJ128MC802__
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_DMA_ADD_INC_4 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN3_ANA & ENABLE_AN5_ANA,
		ENABLE_ALL_DIG_16_31,
		SCAN_NONE_16_31,
		SKIP_SCAN_AN2 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#elif __dsPIC33EP256MC502__
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_SSRC_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_DMA_ADD_INC_4 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_BUF_LOC_1,
		0, // Don't read any pins in porta
		ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN3_ANA & ENABLE_AN5_ANA, // Enable our specific pins
		0, // Don't read any pins in portc
		0, // Don't read any pins in portd
		0, // Don't read any pins in porte
		0, // Don't read any pins in portf
		0, // Don't read any pins in portg
		0, // Don't read any pins in porth
		0, // Don't read any pins in porti
		0, // Don't read any pins in portj
		0, // Don't read any pins in portk
		SCAN_NONE_16_31,
		SKIP_SCAN_AN2 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#endif
	// Open DMA1 for receiving ADC values
	OpenDMA1(DMA1_MODULE_ON & DMA1_SIZE_WORD & PERIPHERAL_TO_DMA1 & DMA1_INTERRUPT_BLOCK & DMA1_NORMAL & DMA1_PERIPHERAL_INDIRECT & DMA1_CONTINUOUS,
		  DMA1_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
		  __builtin_dmaoffset(adcDmaBuffer),
#elif __dsPIC33EP256MC502__
		  (unsigned long int)adcDmaBuffer,
#endif
		  NULL,
		  (uint16_t)&ADC1BUF0,
		  3); // Specify the number of pins being measured (n) as n-1 here. Must match ADC_DMA_ADD_INC_n setting.
	DMA1REQbits.IRQSEL = 0x0D; // Attach this DMA to the ADC1 conversion done event
}
