#ifndef _ECAN_RUDDER_H_
#define _ECAN_RUDDER_H_

#include "stdbool.h"

/**
 * Schedule the CAN messages.
 */
void RudderEcanInit(void);

/**
 * Transmit the appropriate messages for this timestep.
 * Relies on the MavLinkMessageScheduler to determine which
 * those are. Actual transmission is deferred to helper
 * functions.
 */
void RudderTransmit(void);

/**
 * Transmit PGN 127245 message.
 */
void RudderSendNmea(void);

/**
 * Transmit CUSTOM_LIMITS message.
 */
void RudderSendCustomLimit(void);

/**
 * Manage the ECAN message system for a given timestamp. This
 * function reads in and processes all received timestamps. It
 * also checks which messages should be transmit for the current
 * timestep and sends those off.
 */
void SendAndReceiveEcan(void);

void UpdateMessageRate(void);

/**
 * Helper function for retrieving whether a calibration should occur.
 * Used to pull this data into Simulink.
 * @returns A boolean for whether or not to calibrate.
 */
bool GetCalibrateMessage(void);

/**
 * Helper function for retrieving what the commanded angle should be.
 * Used to pull this data into Simulink.
 * @returns The desired angle in radians.
 */
float GetNewAngle(void);

#endif // _ECAN_RUDDER_H_
