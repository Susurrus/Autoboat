#ifndef RC_NODE_H
#define RC_NODE_H

#include <stdbool.h>

/// Store some 16-bit bitfields for tracking the system status and various erros.
/// These have been declared in Node.h
// status tracks various statuses of the node.
//  * bit 0: Manual control enabled.
enum RC_NODE_STATUS {
	RC_NODE_STATUS_MANUAL_MODE = 0x0001
};

// errors tracks the various flags that can put the node into a reset state.
//  * bit 0: eStop has been pushed.
//  * bit 1: RC node is uncalibrated  (therefore also active during calibration)
//  * bit 2: the ECAN peripheral has reached an error state for transmission
//  * bit 3: the ECAN peripheral has reached an error state for reception
enum RC_NODE_RESET {
	RC_NODE_RESET_ESTOP        = 0x0001,
	RC_NODE_RESET_UNCALIBRATED = 0x0002,
	RC_NODE_RESET_ECAN_TX_ERR  = 0x0004,
	RC_NODE_RESET_ECAN_RX_ERR  = 0x0008
};

// Store the calibrated ranges for the rudder and throttle channels
extern uint16_t rcRudderRange[2];
extern uint16_t rcThrottleRange[2];

// Track if the node restored the calibration values on startup
extern bool restoredCalibration;

// Track if the e-stop is active or not.
extern bool estopActive;

void RcNodeInit(void);

bool GetEstopStatus(void);

uint8_t ProcessAllEcanMessages(void);

#endif // RC_NODE_H