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
#include <math.h>

#include <pps.h>

#include "Node.h"
#include "Uart1.h"
#include "MessageScheduler.h"
#include "Types.h"
#include "Ecan1.h"
#include "CanMessages.h"
#include "Tokimec.h"
#include "ImuNode.h"

/**
 * Store specific data output from the Tokimec. We can use this both as the intermediate data store
 * for the Tokimec parser as well as the global data store because it will never be read or written
 * while the other operation is happening because none of that is done during interrupts.
 */
static TokimecOutput tokimecData = {};

// Specify the sensor timeout to be .2s if the checking is run at 100Hz.
#define SENSOR_TIMEOUT 20
typedef struct {
	bool enabled            : 1; // If the sensor is enabled, i.e. it is online and transmitting messages.
	uint8_t enabled_counter : 7; // The timeout counter for this sensor being enabled.
	bool active             : 1; // If the sensor is active, i.e. receiving valid data.
	uint8_t active_counter  : 7; // The timeout counter for this sensor being active.
} timeoutCounters;


// Start with the assumption that we're disconnected and trigger events starting up as needed.
struct stc {
	timeoutCounters imu;
} sensorAvailability = {
	{0, SENSOR_TIMEOUT, 0, SENSOR_TIMEOUT}
};


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

// Specify the frequency in Hz that these tasks should be executed at. Used with the message scheduler
// library.
#define RATE_TRANSMIT_NODE_STATUS      2
#define RATE_TRANSMIT_IMU_DATA        25
#define RATE_TRANSMIT_BLINK_DEFAULT    1
#define RATE_TRANSMIT_BLINK_CONNECTED  4

void ImuNodeInit(uint32_t f_osc)
{
    // And configure the Peripheral Pin Select pins:
    PPSUnLock;
	PPSUnLock;

#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP4);

	// To enable UART1 pins: TX on 9, RX on 8
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP9);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP8);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP36);

	// To enable UART1 pins: TX on 41, RX on 40
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP41);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP40);
#endif

    PPSLock;
	
	// Also disable analog functionality on B8 so we can use it for UART1 RX.
	// This only applies to the dsPIC33E family.
#ifdef __dsPIC33EP256MC502__
	ANSELBbits.ANSB8 = 0;
#endif

    // Initialize status LEDs for use.
	// A3 (output): Red LED, off by default, and is solid when the system hit a fatal error.
    _TRISA3 = 0; 
    _LATA3 = 0;
	// A4 (output): Amber LED, blinks at 1Hz when disconnected from the IMU, 2Hz otherwise.
    _TRISA4 = 0;
    _LATA4 = 0;

	_TRISB7 = 0; // Set ECAN1_TX pin to an output
	_TRISB4 = 1; // Set ECAN1_RX pin to an input;

    // Set up UART1 for 115200 baud. There's no round() on the dsPICs, so we implement our own.
	double brg = (double)f_osc / 2.0 / 16.0 / 115200.0 - 1.0;
	if (brg - floor(brg) >= 0.5) {
		brg = ceil(brg);
	} else {
		brg = floor(brg);
	}
	Uart1Init((uint16_t)brg);

    // Initialize ECAN1 for input and output using DMA buffers 0 & 2
    Ecan1Init(f_osc);

    // Set the node ID
    nodeId = CAN_NODE_IMU_SENSOR;

    // Set up all of our tasks.
    // Blink at 1Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_BLINK, RATE_TRANSMIT_BLINK_DEFAULT)) {
            FATAL_ERROR();
    }
    // Transmit node status at 2Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_STATUS, RATE_TRANSMIT_NODE_STATUS)) {
            FATAL_ERROR();
    }
    // Transmit IMU data at 25Hz
    if (!AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_IMU, RATE_TRANSMIT_IMU_DATA)) {
            FATAL_ERROR();
    }
}

/**
 * This function reads in new data from UART1 and feeds it into our TokimecParser.
 */
void RunContinuousTasks(void)
{

	uint8_t c;
	while (Uart1ReadByte(&c)) {
		// If we've successfully decoded a message...
		if (TokimecParse((char)c, &tokimecData) > 0) {
			// Log that the IMU is connected.
			sensorAvailability.imu.enabled_counter = 0;
			sensorAvailability.imu.active_counter = 0;
		}
	}
}

void Run100HzTasks(void)
{
    // Track the tasks to be performed for this timestep.
    static uint8_t msgs[NUM_TASKS];
	
	// Increment sensor availability timeout counters.
	if (sensorAvailability.imu.enabled_counter < SENSOR_TIMEOUT) {
		++sensorAvailability.imu.enabled_counter;
	}
	if (sensorAvailability.imu.active_counter < SENSOR_TIMEOUT) {
		++sensorAvailability.imu.active_counter;
	}

    uint8_t messagesToSend = GetMessagesForTimestep(&taskSchedule, msgs);
    int i;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case TASK_TRANSMIT_IMU: {
                CanMessage msg;

				// Transmit the absolute attitude message
                CanMessagePackageImuData(&msg,
                                         tokimecData.nice.yaw,
                                         tokimecData.nice.pitch,
                                         tokimecData.nice.roll);
                Ecan1Transmit(&msg);

				// Now transmit the angular velocity data
                CanMessagePackageAngularVelocityData(&msg,
                                         tokimecData.nice.x_angle_vel,
                                         tokimecData.nice.y_angle_vel,
                                         tokimecData.nice.z_angle_vel);
                Ecan1Transmit(&msg);

				// And then the accelerometer data
                CanMessagePackageAccelerationData(&msg,
                                         tokimecData.nice.x_accel,
                                         tokimecData.nice.y_accel,
                                         tokimecData.nice.z_accel);
                Ecan1Transmit(&msg);

				// And now the position data
                CanMessagePackageGpsPosData(&msg,
                                         tokimecData.nice.latitude,
                                         tokimecData.nice.longitude);
                Ecan1Transmit(&msg);

				// And its estimated position data
                CanMessagePackageEstGpsPosData(&msg,
                                         tokimecData.nice.est_latitude,
                                         tokimecData.nice.est_longitude);
                Ecan1Transmit(&msg);

				// And finally a few random data bits
                CanMessagePackageGpsVelData(&msg,
                                         tokimecData.nice.gpsDirection,
                                         tokimecData.nice.gpsSpeed,
                                         tokimecData.nice.magneticBearing,
                                         tokimecData.nice.status);
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
	
	// And update sensor availability.
	if (sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter >= SENSOR_TIMEOUT) {
		sensorAvailability.imu.enabled = false;

		// When the IMU is no longer connected, blink at a regular rate.
		RemoveMessage(&taskSchedule, TASK_BLINK);
		AddMessageRepeating(&taskSchedule, TASK_BLINK, RATE_TRANSMIT_BLINK_DEFAULT);

		// When the IMU is no longer connected, no longer transmit IMU messages.
		RemoveMessage(&taskSchedule, TASK_TRANSMIT_IMU);

		// Also update our status.
		nodeStatus &= ~IMU_NODE_STATUS_FLAG_IMU_ACTIVE;
	} else if (!sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter < SENSOR_TIMEOUT) {
		sensorAvailability.imu.enabled = true;

		// When the IMU is connected, blink a little faster.
		RemoveMessage(&taskSchedule, TASK_BLINK);
		AddMessageRepeating(&taskSchedule, TASK_BLINK, RATE_TRANSMIT_BLINK_CONNECTED);
		
		// When the IMU is reconnected, transmit IMU messages.
		AddMessageRepeating(&taskSchedule, TASK_TRANSMIT_IMU, RATE_TRANSMIT_IMU_DATA);

		// Also update our status.
		nodeStatus |= IMU_NODE_STATUS_FLAG_IMU_ACTIVE;
	}
}
