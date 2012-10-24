#ifndef _ECAN_RUDDER_H_
#define _ECAN_RUDDER_H_

#include "stdbool.h"

/**
 * Initialize the rudder subsystem. Currently this means
 * scheduling repetitive transmission of CAN messages.
 */
void RudderSubsystemInit(void);

/**
 * Transmit PGN 127245 message via CAN.
 */
void RudderSendNmea(void);

/**
 * Transmit CUSTOM_LIMITS message via CAN.
 */
void RudderSendCustomLimit(void);

/**
 * Manage the ECAN message system for a given timestamp. This
 * function reads in and processes all received timestamps. It
 * also checks which messages should be transmit for the current
 * timestep and sends those off.
 */
void SendAndReceiveEcan(void);

/**
 * Update the transmission rates of broadcast messages.
 * @param angleRate The desired transmission rate of the NMEA2000 PGN127245 message.
 * @param statusRate The desired transmission rate of the custom status message.
 */
void UpdateMessageRate(const uint8_t angleRate, const uint8_t statusRate);

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
