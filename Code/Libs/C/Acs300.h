#ifndef ACS300_H
#define ACS300_H

/**
 * @file
 * Thie library provides an interface for the ACS300 brushless motor driver board by Automotion Inc.
 * Note that all CAN messages are in big-endian format.
 */

#include "EcanDefines.h"

#include <stdint.h>

// Declare the base address used for incoming CAN messages
#define ACS300_BASE_CN_RA        0x300
#define ACS300_CAN_ID_VEL_CMD    ACS300_BASE_CN_RA
#define ACS300_CAN_ID_WR_PARAM  (ACS300_BASE_CN_RA + 1)
#define ACS300_CAN_ID_RD_PARAM  (ACS300_BASE_CN_RA + 2)

// Declare the base address used for outgoing CAN messages
#define ACS300_BASE_CN_TA        0x400
#define ACS300_CAN_ID_ERR_RSP    ACS300_BASE_CN_TA
#define ACS300_CAN_ID_PARAM_RSP (ACS300_BASE_CN_TA + 1)
#define ACS300_CAN_ID_HRTBT     (ACS300_BASE_CN_TA + 2)

// Declare the various parameters onboard
#define ACS300_PARAM_CC 0x0105

// Declare the command bits for use with the VelocityCommand message. These can be bitwise-ORed
// together to combine multiple commands in certain cases.
enum ACS300_COMMAND {
	ACS300_COMMAND_CLEAR_FAULTS     = 0x8000,
	ACS300_COMMAND_STANDBY          = 0x4000,
	ACS300_COMMAND_RUN              = 0x2000,
	ACS300_COMMAND_WRITE_TO_EEPROM  = 0x1000,
	ACS300_COMMAND_READ_FROM_EEPROM = 0x0800
};

// Declare the status bits returned from the Heartbeat. Currently incomplete as I don't need every
// value.
enum ACS300_STATUS {
	ACS300_STATUS_BOOTING = 0x8000,
	ACS300_STATUS_STANDBY = 0x4000
};

// The following two functions apply to ACS300_CAN_ID_VEL_CMD
void Acs300PackageVelocityCommand(CanMessage *msg, int16_t torqueFeedForward, int16_t velCommand, uint16_t status);

void Acs300DecodeVelocityCommand(const uint8_t data[6], int16_t *torqueFeedForward, int16_t *velCommand, uint16_t *status);

// The following two functions apply to ACS300_CAN_ID_WR_PARAM
void Acs300PackageWriteParam(CanMessage *msg, uint16_t address, uint16_t value);

void Acs300DecodeWriteParam(const uint8_t data[4], uint16_t *address, uint16_t *value);

// The following two functions apply to ACS300_CAN_ID_HRTBT
void Acs300PackageHeartbeat(CanMessage *msg, uint16_t dataA, uint16_t dataB, uint16_t voltage, uint16_t errorStatus);

void Acs300DecodeHeartbeat(const uint8_t data[8], uint16_t *dataA, uint16_t *dataB, uint16_t *voltage, uint16_t *errorStatus);

#endif // ACS300_H