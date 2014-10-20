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
// Include standard headers
#include <math.h>
#include <stdio.h>
#include <string.h>

// Include Microchip headers
#include <pps.h>

// Include user headers
#include "Node.h"
#include "Uart1.h"
#include "MessageScheduler.h"
#include "Types.h"
#include "Ecan1.h"
#include "CanMessages.h"
#include "AttitudeNode.h"
#include "MPU60xx.h"
#include "I2CdsPIC.h"

#ifndef __dsPIC33EP256MC502__
#error Must use a dsPIC33EP256MC502
#endif

MPU6050_Data imu_data;

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

void AttitudeNodeInit(uint32_t f_osc)
{
    // And configure the Peripheral Pin Select pins:
    PPSUnLock;

    // To enable ECAN1 pins: TX on 39, RX on 36
    PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
    PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP36);

    // Enable interrupt 1 on RB1
    PPSInput(IN_FN_PPS_INT1, IN_PIN_PPS_RPI33);

    // FIXME: Remove this after debugging.
    // To enable UART1 pins: TX on 43 (B11), MAVLink input decode on 43 (B11), RX on 45 (B13)
    PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP43);
    PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RPI45);

    PPSLock;

    // Also disable analog functionality on B8 so we can use the i2c
    ANSELBbits.ANSB8 = 0;

    // And disable analog functionality on B1 for using the interrupt peripheral
    ANSELBbits.ANSB1 = 0;

    // i2c pins are bidirectional and don't need their TRIS bits messed with

    // Initialize status LEDs for use.
    // A3 (output): Red LED, on by default, indicates an error.
    // Once the devices finishes startup, it'll turn off the red LED.
    _TRISA3 = 0;
    _LATA3 = 1;
    // A4 (output): Amber LED, blinks at 1Hz when disconnected from the IMU, 2Hz otherwise.
    ANSELAbits.ANSA4 = 0; // Need to disable analog functionality on it
    _TRISA4 = 0;
    _LATA4 = 0;

    _TRISB7 = 0; // Set ECAN1_TX pin to an output
    _TRISB4 = 1; // Set ECAN1_RX pin to an input;
    _TRISB0 = 1; // Set INT1 pin to an input

    // Set up UART1 for 115200 baud. There's no round() on the dsPICs, so we implement our own.
    double brg = (double)f_osc / 2.0 / 16.0 / 115200.0 - 1.0;
    if (brg - floor(brg) >= 0.5) {
        brg = ceil(brg);
    } else {
        brg = floor(brg);
    }
    Uart1Init((uint16_t)brg);

    // Initialize ECAN1 for input and output using DMA buffers 0 & 2
    Ecan1Init(f_osc, NODE_CAN_BAUD);

    // Bring up the i2c peripheral bus at 400khz
    I2C_Init(I2C_CALC_BRG(400000, f_osc));

    // Initialize the MPU6050, enabling slave devices for the future addition of
    // the MAG3110.
    MPU60xx_Init(true);

    // Set the node ID
    nodeId = CAN_NODE_ATTITUDE_SENSOR;

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
}

void Run100HzTasks(void)
{
    // Track the tasks to be performed for this timestep.
    static uint8_t msgs[NUM_TASKS];

    // Get the IMU data
    MPU60xx_Get6AxisData(&imu_data);

    // And output the IMU data over UART for debugging.
    char imuDataStr[100];
    sprintf(imuDataStr, "Accel: (%d, %d, %d), Gyro: (%d, %d, %d)\n",
            imu_data.accelX, imu_data.accelY, imu_data.accelZ,
            imu_data.gyroX, imu_data.gyroY, imu_data.gyroZ);
    Uart1WriteData(imuDataStr, strlen(imuDataStr));

    uint8_t messagesToSend = GetMessagesForTimestep(&taskSchedule, msgs);
    int i;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case TASK_TRANSMIT_IMU: {
//                CanMessage msg;

                // TODO: Transmit quaternion data

//                Ecan1Transmit(&msg);
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
