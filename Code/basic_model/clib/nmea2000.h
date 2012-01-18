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

// Units are seqId: none, waterSpeed: m/s.
unsigned char ParsePgn128259(unsigned char data[8], unsigned char *seqId, float *waterSpeed);

// Units are seqId: none, waterDepth: m, offset: m.
unsigned char ParsePgn128267(unsigned char data[8], unsigned char *seqId, float *waterDepth, float *offset);

// Units are seqId: none, latitude: degrees, longitude: degrees.
unsigned char ParsePgn129025(unsigned char data[8], float *latitude, float *longitude);

// Units are seqId: none, cogRef: 0=True, 1=magnetic, cog: .0001 rads eastward from north, sog: .01 m/s
unsigned char ParsePgn129026(unsigned char data[8], unsigned char *seqId, unsigned char *cogRef, float *cog, float *sog);

// Units are obvious.
unsigned char ParsePgn129033(unsigned char data[8], unsigned char *day, unsigned char *month, unsigned char *year, unsigned char *hour, unsigned char *minute, unsigned char *seconds);

// Units are seqId: none, airSpeed: m/s, direction: radians eastward from north.
unsigned char ParsePgn130306(unsigned char data[8], unsigned char *seqId, float *airSpeed, float *direction);

// Units are seqId: none, waterTemp: degrees C, airTemp: degrees C, and airPressure: kPa.
unsigned char ParsePgn130310(unsigned char data[8], unsigned char *seqId, float *waterTemp, float *airTemp, float *airPressure);

// Units are seqId: none, temp: degrees C, humidity: %, pressure: kPa.
unsigned char ParsePgn130311(unsigned char data[8], unsigned char *seqId, float *temp, float *humidity, float *pressure);

#endif // __NMEA2000_H__
