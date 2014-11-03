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
#include <stdlib.h>
#include <string.h>

// Include Microchip headers
#include <pps.h>

// Include user headers
#include "AttitudeNode.h"
#include "CanMessages.h"
#include "Ecan1.h"
#include "IMU.h"
#include "MessageScheduler.h"
#include "Node.h"
#include "Packing.h"
#include "Types.h"
#include "Uart1.h"

#ifndef __dsPIC33EP256MC502__
#error Must use a dsPIC33EP256MC502
#endif

static MPU6050_Data mpuData;
static MAG3110_Data magData;
static IMU_Data imuData;

// Track when new IMU data has arrived. This is set in a change notification
// interrupt and then continuously read in the main loop.
static bool newImuData = false;

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

    // i2c pins are bidirectional and don't need their TRIS bits messed with

    // Set up RB1 for change notifications so that we don't constantly poll the
    // I2C lines.
    ANSELBbits.ANSB1 = 0; // Set to a digital pin
    _TRISB1 = 1; // Set to a digital input
    CNENBbits.CNIEB1 = 1; // Turn on the change notification for this pin
    IEC1bits.CNIE = 1; // Turn on change notification interrupts
    IFS1bits.CNIF = 0; // Reset the change notification interrupt

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

    // Bring up the I2C bus at 400kHz and the attached MPU-6050/MAG3110 devices.
    IMU_Init(400000, f_osc);

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
    // Get the IMU data if a new data pulse has been detected and feed it into
    // the IMU AHRS algorithm.
    if (newImuData) {
        IMU_GetData(&mpuData, &magData);
        IMU_normalizeData(mpuData, magData, &imuData);
        newImuData = false;
    }
}

void Run100HzTasks(void)
{
    // Track the tasks to be performed for this timestep.
    static uint8_t msgs[NUM_TASKS];

    IMU_UpdateAHRS(&imuData);

    // And output the IMU data over UART for debugging.
    // We support human-readable or plottable data.
    char imuDataStr[100];
#define OUTPUT 2
#if OUTPUT == 0
    sprintf(imuDataStr, "Accel: (%d, %d, %d), Gyro: (%d, %d, %d), Mags: (%d, %d, %d)\n",
            imuData.accelX, imuData.accelY, imuData.accelZ,
            imuData.gyroX, imuData.gyroY, imuData.gyroZ,
            magData.magX, magData.magY, magData.magZ);
    Uart1WriteData(imuDataStr, strlen(imuDataStr));
#elif OUTPUT == 1
    float q[4];
    IMU_GetQuaternion(q);
    sprintf(imuDataStr, "%f,%f,%f,%f\n", (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
    Uart1WriteData(imuDataStr, strlen(imuDataStr));
#elif OUTPUT == 2
    float q[4];
    IMU_GetQuaternion(q);
    float ypr[3];
    IMU_QuaternionToYawPitchRoll(q, ypr);

    // Convert to fixed-point value with precision of 1/ten-thousandths of a radian
    int16_t yaw = (int16_t)(ypr[0] * 10000.0);
    int16_t pitch = (int16_t)(ypr[1] * 10000.0);
    int16_t roll = (int16_t)(ypr[2] * 10000.0);
    itoa(imuDataStr, yaw, 10);
    size_t newIndex = strlen(imuDataStr);
    imuDataStr[newIndex++] = ',';
    itoa(&imuDataStr[newIndex], pitch, 10);
    newIndex = strlen(imuDataStr);
    imuDataStr[newIndex++] = ',';
    itoa(&imuDataStr[newIndex], roll, 10);
    newIndex = strlen(imuDataStr);
    imuDataStr[newIndex++] = '\n';
    Uart1WriteData(imuDataStr, newIndex);
#elif OUTPUT == 3
    float q[4];
    IMU_GetQuaternion(q);
    IMU_QuaternionToString(q, imuDataStr);
    Uart1WriteData(imuDataStr, 37);
#endif

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

void _ISR _CNInterrupt(void)
{
    // If pin B1 is high, it means that this interrupt was the rising edge of the
    // pin.
    if (_RB1 == 1) {
        newImuData = true;
    }

    IFS1bits.CNIF = 0; // Clear the interrupt
}