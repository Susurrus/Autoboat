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

#include <pps.h>

#include "Node.h"
#include "Uart1.h"
#include "MessageScheduler.h"
#include "Types.h"
#include "Ecan1.h"
#include "CanMessages.h"
#include "Tokimec.h"

/**
 * Store specific data output from the Tokimec.
 */
static struct {
    float roll; // In rads
    float pitch; // In rads
    float yaw; // In rads
} tokimecData;

// Set up the message scheduler for running 3 tasks:
//  * Blinking the status LED at 1Hz
//  * Transmitting the node status at 2Hz
//  * Transmitting power status at 10Hz
#define NUM_TASKS 3
enum {
	TASK_BLINK = 1,
	TASK_TRANSMIT_STATUS,
	TASK_TRANSMIT_IMU
};
uint8_t taskIds[NUM_TASKS] = {
	TASK_BLINK,
	TASK_TRANSMIT_STATUS,
	TASK_TRANSMIT_IMU
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

void ImuNodeInit(uint32_t f_osc)
{
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

    // Initialize status LEDs for use.
    _TRISA3 = 0;
    _TRISA4 = 0;
    _LATA3 = 0;
    _LATA4 = 0;

    // Set up UART1 for 115200 baud.
    Uart1Init((uint16_t)((f_osc / 2l) / (16l * 115200l) - 1l));

    // Initialize ECAN1 for input and output using DMA buffers 0 & 2
    Ecan1Init(f_osc);

    // Set the node ID
    nodeId = CAN_NODE_IMU_SENSOR;

    // Set up all of our tasks.
    // Blink at 1Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_BLINK, 1)) {
            FATAL_ERROR();
    }
    // Transmit node status at 2Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_STATUS, 2)) {
            FATAL_ERROR();
    }
    // Transmit IMU data at 25Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_IMU, 25)) {
            FATAL_ERROR();
    }
}

/**
 * This function reads in new data from UART2 and feeds it into the NMEA0183 parser which then
 * calls the Revo GS library for parsing the data out.
 */
void RunContinuousTasks(void)
{
	// The following variables are all necessary for the NMEA0183 parsing of the RevoGS messages.
	static TokimecOutput o;

        uint8_t c;
	while (Uart1ReadByte(&c)) {
		TokimecParse((char)c, &o);

                // Convert the incoming 3-axis data to straight radians.
                tokimecData.roll = (float)o.nice.roll/8192.0;
                tokimecData.pitch = (float)o.nice.pitch/8192.0;
                tokimecData.yaw = (float)o.nice.yaw/8192.0;
	}
}

void Run100HzTasks(void)
{
    // Track the tasks to be performed for this timestep.
    static uint8_t msgs[NUM_TASKS];

    uint8_t messagesToSend = GetMessagesForTimestep(&taskSchedule, msgs);
    int i;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case TASK_TRANSMIT_IMU: {
                CanMessage msg;
                CanMessagePackageImuData(&msg,
                                         tokimecData.yaw,
                                         tokimecData.pitch,
                                         tokimecData.roll);
                Ecan1Transmit(&msg);
            } break;
            case TASK_TRANSMIT_STATUS:
                NodeTransmitStatus();
            break;
            case TASK_BLINK: // Blink the status LED at 1Hz
                    _LATA4 ^= 1;
            break;
        }
    }
}
