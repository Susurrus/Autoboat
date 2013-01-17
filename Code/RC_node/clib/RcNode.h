#ifndef RC_NODE_H
#define RC_NODE_H

/// Store some 16-bit bitfields for tracking the system status and various erros.
/// These have been declared in Node.h
// status tracks various statuses of the node.
//  * bit 0: Manual control enabled.

// errors tracks the various flags that can put the node into a reset state.
//  * bit 0: eStop has been pushed.
//  * bit 1: RC node is uncalibrated  (therefore also active during calibration)
//  * bit 2: the ECAN peripheral has reached an error state for transmission
//  * bit 3: the ECAN peripheral has reached an error state for reception

void RcNodeInit(void);

bool GetEstopStatus(void);

void SendThrottleCommand(int16_t command);

/**
 * This function restored the calibrated range for the RC receiver PWM signals if any exist.
 * Since the EEPROM is initialized to 0xFFFF for each uint16 we only restore the values if
 * the memory locations that should contain our data just contain 0xFFFF instead. restoredCalibration
 * is another exported global that is just a Boolean for whether or not we restored saved data. This
 * is used to correct the calibration state used elsewhere.
 */
void InitCalibrationRange(void);

uint8_t ProcessAllEcanMessages(void);

#endif // RC_NODE_H