#ifndef __NMEA2000_H__
#define __NMEA2000_H__

/**
 * Miscellaneous utilities.
 */

// Converts the 29-bit CAN extended address into its components according to the ISO 11783 spec.
unsigned long ISO11783Decode(unsigned long id, unsigned char *src, unsigned char *dest, unsigned char *pri);

/***
 * PGN Parsers
 */

// Units are m/s.
unsigned char ParsePgn128259(unsigned char data[8], unsigned char *seqId, float *waterSpeed);

// Units are m, m.
unsigned char ParsePgn128267(unsigned char data[8], unsigned char *seqId, float *waterDepth, float *offset);

// Units are m/s, radians eastward from north.
unsigned char ParsePgn130306(unsigned char data[8], unsigned char *seqId, float *airSpeed, float *direction);

// Units are deg C, deg C, and kPa.
unsigned char ParsePgn130310(unsigned char data[8], unsigned char *seqId, float *waterTemp, float *airTemp, float *airPressure);

// Units are degrees C, %, kPa.
unsigned char ParsePgn130311(unsigned char data[8], unsigned char *seqId, float *temp, float *humidity, float *pressure);

#endif // __NMEA2000_H__
