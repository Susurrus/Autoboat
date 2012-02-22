#ifndef __NMEA2000_H__
#define __NMEA2000_H__

#include <inttypes.h>

/**
 * Miscellaneous utilities.
 */

/**
  * Converts the 29-bit CAN extended address into its components according to the ISO 11783 spec.
  * @param[in] can_id The 29-bit CAN identifier to decode.
  * @param[out] src The source of this message.
  * @param[out] dest The destination for this message. Set to 255 if the message was a broadcast.
  * @param[out] pri The priority, a 3-bit number with higher values indicating higher priority.
  */
uint32_t Iso11783Decode(uint32_t can_id, uint8_t *src, uint8_t *dest, uint8_t *pri);

/**
  * Encodes a 29-bit CAN header given the necessary parameters. Note that there are two
  * separate styles of encoding: PDU1 is a one-to-one communication with a destination,
  * PDU2 is a broadcast without a destination. As the destination and PGN can both encode
  * this information and given the expected usage of this function, the PGN is given
  * priority for determining this.
  * @return The encoded 29-bit CAN identifier (as a uint32)
  */
uint32_t Iso11783Encode(uint32_t pgn, uint8_t src, uint8_t dest, uint8_t pri);

// Given a number of days return the offset since January 1st 1970 in years, months, and days.
void DaysSinceEpochToOffset(uint16_t days, uint8_t *offset_years, uint8_t *offset_months, uint8_t *offset_days);


/***
 * PGN Parsers
 */

 // Units are year: absolute, month: 1-12, day: 1-31, hour: 0-23, min: 0-59, sec: 0-59, seqId: none, source: 0 (GPS), 1 (GLONASS), 2 (Radio station), 3 (Local cesium clock), 4 (local rubidium clock), 5 (Local crystal clock).
 // NOTE: The date values aren't properly decoded yet.
uint8_t ParsePgn126992(uint8_t data[8], uint8_t *seqId, uint8_t *source, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second);

/**
  * Decodes PGN 127508 - Battery Status
  * @param[in] data The 8 payload bytes of the CAN message.
  * @param[out] instance A unique ID for this battery instance.
  * @param[out] voltage The voltage in Volts of this battery. If invalid is NaN.
  * @param[out] current The current in Amperes being drawn from this battery. If invalid is NaN.
  * @param[out] temperature The current temperature in degrees Celsius of the battery. If invalid is NaN.
  * @param[out] seqId The sequence ID for this data. Other messages from the same source sharing this sequence ID involves data measurements from the same timestep.
  * @return A bitfield containing success (1) or failure (0) for data from each output argument. For success to be indicated the corresponding argument must be non-null and have valid data within the message.
  */
uint8_t ParsePgn127508(uint8_t data[8], uint8_t *seqId, uint8_t *instance, float *voltage, float *current, float *temperature);

// Units are seqId: none, waterSpeed: m/s.
uint8_t ParsePgn128259(uint8_t data[8], uint8_t *seqId, float *waterSpeed);

// Units are seqId: none, waterDepth: m, offset: m.
uint8_t ParsePgn128267(uint8_t data[8], uint8_t *seqId, float *waterDepth, float *offset);

// Units are seqId: none, latitude: radians (+ north), longitude: radians (+ east).
uint8_t ParsePgn129025(uint8_t data[8], float *latitude, float *longitude);

// Units are seqId: none, cogRef: 0 (True), 1 (magnetic), cog: radians eastward from north, sog: m/s
uint8_t ParsePgn129026(uint8_t data[8], uint8_t *seqId, uint8_t *cogRef, float *cog, float *sog);

// Units are seqId: none, airSpeed: m/s, direction: radians eastward from north.
uint8_t ParsePgn130306(uint8_t data[8], uint8_t *seqId, float *airSpeed, float *direction);

// Units are seqId: none, waterTemp: degrees C, airTemp: degrees C, and airPressure: kPa.
uint8_t ParsePgn130310(uint8_t data[8], uint8_t *seqId, float *waterTemp, float *airTemp, float *airPressure);

// Units are seqId: none, tempInstance: none/enum, humidityInstance: none/enum, temp: degrees C, humidity: %, pressure: kPa.
uint8_t ParsePgn130311(uint8_t data[8], uint8_t *seqId, uint8_t *tempInstance, uint8_t *humidityInstance, float *temp, float *humidity, float *pressure);

#endif // __NMEA2000_H__
