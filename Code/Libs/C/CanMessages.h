#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

/**
 * This module defines all of the custom CAN messages used within the SeaSlug project. These messages all use the standard CAN ID size (11-bits).
 * Usage of this code involves calling one of the `CanMessagePackage()` functions and then feeding
 * the resultant struct into the ECAN transmission library.
 */

#include <stdint.h>
#include <stdbool.h>
#include "EcanDefines.h"

 // Define the standard (11-bit) IDs for all custom CAN messages.
 // We make sure not to trample on the 300/301/302/400/401/402 messages used by the ACS300.
enum {
    // Rudder messages
    CAN_MSG_ID_RUDDER_DETAILS      = 0x080,
    CAN_MSG_ID_RUDDER_SET_STATE    = 0x081,
    CAN_MSG_ID_RUDDER_SET_TX_RATE  = 0x082,

    // General messages
    CAN_MSG_ID_STATUS              = 0x090,

    // IMU messages (defined by the VSAS-2GM)
    CAN_MSG_ID_IMU_DATA            = 0x102,

    // Gyro messages (for use with Z-only gyro)
    CAN_MSG_ID_GYRO_DATA            = 0x105
};

// Define the length of all of the custom CAN messages.
enum {
    // Rudder messages
    CAN_MSG_SIZE_RUDDER_DETAILS      = 7,
    CAN_MSG_SIZE_RUDDER_SET_STATE    = 1,
    CAN_MSG_SIZE_RUDDER_SET_TX_RATE  = 2,

    // General messages
    CAN_MSG_SIZE_STATUS              = 8,

    // IMU messages (defined by the VSAS-2GM)
    CAN_MSG_SIZE_IMU_DATA            = 6,

    // Gyro messages (for use with Z-only gyro)
    CAN_MSG_SIZE_GYRO_DATA           = 4
};

/**
 * Package the data that makes up a STATUS CAN message.
 * @param cpuLoad This is in units of percent. 0-100 are valid, which any other value invalid. Use 255 (0xFF) to specify an invalid value.
 */
void CanMessagePackageStatus(CanMessage *msg, uint8_t nodeId, uint8_t cpuLoad, int8_t temp, uint8_t voltage, uint16_t status, uint16_t errors);

void CanMessageDecodeStatus(const CanMessage *msg, uint8_t *nodeId, uint8_t *cpuLoad, int8_t *temp, uint8_t *voltage, uint16_t *status, uint16_t *errors);
/**
 * Package the data that makes up a RUDDER_SET_STATE message into a struct suitable for transmission.
 */
void CanMessagePackageRudderSetState(CanMessage *msg, bool enable, bool reset, bool calibrate);

void CanMessageDecodeRudderSetState(const CanMessage *msg, bool *enable, bool *reset, bool *calibrate);

void CanMessageDecodeRudderSetTxRate(const CanMessage *msg, uint16_t *angleRate, uint16_t *statusRate);

void CanMessagePackageRudderDetails(CanMessage *msg, uint16_t potVal, uint16_t portLimitVal, uint16_t sbLimitVal, bool portLimitTrig, bool sbLimitTrig, bool enabled, bool calibrated, bool calibrating);

void CanMessageDecodeRudderDetails(const CanMessage *msg, uint16_t *potVal, uint16_t *portLimitVal, uint16_t *sbLimitVal, bool *portLimitTrig, bool *sbLimitTrig, bool *enabled, bool *calibrated, bool *calibrating);

// The IMU data messages are based on the Direction/Attitude messages from the VSAS-2GM, which uses
// a big-endian storage format. All units are in radians.
void CanMessagePackageImuData(CanMessage *msg, float direction, float pitch, float roll);

void CanMessageDecodeImuData(const CanMessage *msg, float *direction, float *pitch, float *roll);

/**
 * The gyro data messages are based on the NMEA2000 PGN127251 message, which indicates vehicle rate. This gyro
 * message doesn't indicate vessel turn rate, however, merely the z-axis turn rate. It uses the little-endian storage format
 * to be consistent with the NMEA2000 standard.
 * @param zRate Rotation rate around the z-axis. Units are in radians/s. Positive indicates clockwise rotation.
 */
void CanMessagePackageGyroData(CanMessage *msg, float zRate);

/**
 * The gyro data messages are based on the NMEA2000 PGN127251 message, which indicates vehicle rate. This gyro
 * message doesn't indicate vessel turn rate, however, merely the z-axis turn rate. It uses the little-endian storage format
 * to be consistent with the NMEA2000 standard.
 * @param zRate Rotation rate around the z-axis. Units are in radians/s. Positive indicates clockwise rotation.
 */
void CanMessageDecodeGyroData(const CanMessage *msg, float *zRate);

#endif // CAN_MESSAGES_H