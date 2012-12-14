#ifndef NMEA2000_ENCODE_H
#define NMEA2000_ENCODE_H

#include "EcanDefines.h"

/**
 *
 * @param msg
 * @param sourceDevice An ID representing the transmitting device.
 * @param instance Which rudder instance this is. Used for when multiple rudder controller exist. If no value or unknown use 0xFF.
 * @param dirOrder UNKNOWN. Set to 0x3 to specify invalid.
 * @param angleOrder Commanded rudder angle in radians.  If no value or invalid use NAN.
 * @param position Sensed rudder angle in radians.  If no value or invalid use NAN.
 */
void PackagePgn127245(CanMessage *msg, uint8_t sourceDevice, uint8_t instance, uint8_t dirOrder, float angleOrder, float position);

/**
 *
 * @param msg
 * @param sourceDevice An ID representing the transmitting device.
 * @param battInstance A unique ID for this battery status. If no value or invalid use 0xFF.
 * @param voltage The voltage measured for this device in Volts. If no value or invalid use NAN.
 * @param amperage The amps measured as being drawn from this device in Amps. If no value or invalid use NAN.
 * @param temp The temperature of this battery measured in degrees C. If no value or invalid use NAN.
 * @param sid The sequence ID for this measurement. If no value or invalid use NAN.
 */
void PackagePgn127508(CanMessage *msg, uint8_t sourceDevice, uint8_t battInstance, float voltage, float amperage, float temp, uint8_t sid);

void PackagePgn129025(CanMessage *msg, uint8_t sourceDevice, int32_t latitude, int32_t longitude);

void PackagePgn129026(CanMessage *msg, uint8_t sourceDevice, uint8_t seqId, uint8_t cogRef, uint16_t cog, uint16_t sog);

/**
 *
 * @param msg The provided struct to store the encoded CAN message in
 * @param sourceDevice An ID representing the transmitting device.
 * @param sid The sequence ID of this data set. If no value or unknown use 0xFF.
 * @param tempInst The instance of this temperature measurement. See PGN_130311_TEMP_INST.
 * @param humidInst The instance of this humidity measurement. See PGN_130311_HUMID_INST.
 * @param temp The temperature reading in Celsius. If no value or invalid use NAN.
 * @param humid The humidity reading in percent. If no value or invalid use NAN.
 * @param press The pressure reading in kPa. If no value or invalid use NAN.
 * @see PGN_130311_TEMP_INST
 * @see PGN_130311_HUMID_INST
 */
void PackagePgn130311(CanMessage *msg, uint8_t sourceDevice, uint8_t sid, uint8_t tempInst, uint8_t humidInst, float temp, float humid, float press);

#endif // NMEA2000_ENCODE_H