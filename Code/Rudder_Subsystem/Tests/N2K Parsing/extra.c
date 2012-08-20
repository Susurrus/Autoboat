#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#include "uart2.h"
#include "ecanDefinitions.h"
#include "nmea2000.h"
#include "types.h"

static struct {
	tFloatToChar speed;
	tFloatToChar direction;
	bool newData;
} windData;

static struct {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool newData;
} airData;

static struct {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool newData;
} waterData;

static struct {
	unsigned char day;
	unsigned char month;
	unsigned char year;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
	bool newData;
} timeAndDateData;

static struct {
	tFloatToChar latitude;
	tFloatToChar longitude;
	bool newData;
} locationData;

static struct {
	tFloatToChar courseOverGround;
	unsigned char courseReference;
	tFloatToChar speedOverGround;
	bool newData;
} courseAndSpeedData;

void GetWindData(unsigned char *data) {
	data[0] = windData.speed.chData[0];
	data[1] = windData.speed.chData[1];
	data[2] = windData.speed.chData[2];
	data[3] = windData.speed.chData[3];
	data[4] = windData.direction.chData[0];
	data[5] = windData.direction.chData[1];
	data[6] = windData.direction.chData[2];
	data[7] = windData.direction.chData[3];
	data[8] = windData.newData;
	windData.newData = false;
}

void GetAirData(unsigned char *data) {
	data[0] = airData.temp.chData[0];
	data[1] = airData.temp.chData[1];
	data[2] = airData.temp.chData[2];
	data[3] = airData.temp.chData[3];
	data[4] = airData.pressure.chData[0];
	data[5] = airData.pressure.chData[1];
	data[6] = airData.pressure.chData[2];
	data[7] = airData.pressure.chData[3];
	data[8] = airData.humidity.chData[0];
	data[9] = airData.humidity.chData[1];
	data[10] = airData.humidity.chData[2];
	data[11] = airData.humidity.chData[3];
	data[12] = airData.newData;
	airData.newData = false;
}

void GetWaterData(unsigned char *data) {
	data[0] = waterData.speed.chData[0];
	data[1] = waterData.speed.chData[1];
	data[2] = waterData.speed.chData[2];
	data[3] = waterData.speed.chData[3];
	data[4] = waterData.temp.chData[0];
	data[5] = waterData.temp.chData[1];
	data[6] = waterData.temp.chData[2];
	data[7] = waterData.temp.chData[3];
	data[8] = waterData.depth.chData[0];
	data[9] = waterData.depth.chData[1];
	data[10] = waterData.depth.chData[2];
	data[11] = waterData.depth.chData[3];
	data[12] = waterData.newData;
	waterData.newData = false;
}

void GetLocationData(unsigned char *data) {
	data[0] = locationData.latitude.chData[0];
	data[1] = locationData.latitude.chData[1];
	data[2] = locationData.latitude.chData[2];
	data[3] = locationData.latitude.chData[3];
	data[4] = locationData.longitude.chData[0];
	data[5] = locationData.longitude.chData[1];
	data[6] = locationData.longitude.chData[2];
	data[7] = locationData.longitude.chData[3];
	data[8] = locationData.newData;
	locationData.newData = false;
}

void GetCourseAndSpeedData(unsigned char *data) {
	data[0] = courseAndSpeedData.courseOverGround.chData[0];
	data[1] = courseAndSpeedData.courseOverGround.chData[1];
	data[2] = courseAndSpeedData.courseOverGround.chData[2];
	data[3] = courseAndSpeedData.courseOverGround.chData[3];
	data[4] = courseAndSpeedData.courseReference;
	data[5] = courseAndSpeedData.speedOverGround.chData[0];
	data[6] = courseAndSpeedData.speedOverGround.chData[1];
	data[7] = courseAndSpeedData.speedOverGround.chData[2];
	data[8] = courseAndSpeedData.speedOverGround.chData[3];
	data[9] = courseAndSpeedData.newData;
	courseAndSpeedData.newData = false;
}

void GetTimeAndDateData(unsigned char *data) {
	data[0] = timeAndDateData.day;
	data[1] = timeAndDateData.month;
	data[2] = timeAndDateData.year;
	data[3] = timeAndDateData.hour;
	data[4] = timeAndDateData.minute;
	data[5] = timeAndDateData.second;
	data[6] = timeAndDateData.newData;
	timeAndDateData.newData = false;
}

void initCommunications() {
	initUart2(42); // Initialize UART2 for 57600 baud.
}

void DisplayWindData(float speed, float direction) {
	char text[120];
	sprintf(text, "Wind data - speed: %2.1f (m/s), dir: %2.1f (rads)\n", speed, direction);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayAirData(float temp, float pressure, float humidity) {
	char text[120];
	sprintf(text, "Air data - temp: %2.1f (deg C), press: %2.1f (kPa), humid: %2.1f (%%)\n", temp, pressure, humidity);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayWaterData(float speed, float temp, float depth){
	char text[120];
	sprintf(text, "Water data - speed: %2.1f (m/s), temp: %2.1f (deg C), depth: %2.1f (m)\n", speed, temp, depth);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayLocationData(float latitude, float longitude){
	char text[120];
	sprintf(text, "Location data - latitude: %2.1f (deg), longitude: %2.1f (deg)\n", latitude, longitude);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayTimeData(unsigned char hour, unsigned char minute, unsigned char second){
	char text[120];
	sprintf(text, "Time data - hour: %d, minute: %d, second: %d\n", hour, minute, second);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayDateData(unsigned char day, unsigned char month, unsigned char year){
	char text[120];
	sprintf(text, "Date data - day: %d, month: %d, year: %d\n", day, month, year);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayCourseAndSpeedData(float courseOverGround, unsigned char courseReference, float speedOverGround){
	char text[120];
	sprintf(text, "Course/speed data - Course (over ground): %2.1f (deg), Course reference: %s, Speed (over ground): %2.1f (m/s)\n", courseOverGround, courseReference?"Mag":"True", speedOverGround);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayUnhandledId(unsigned long pgn, unsigned char data[8]) {
	char text[120];

	sprintf(text, "PGN %lu (%02x %02x %02x %02x %02x %02x %02x %02x) - Unprocessed\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], pgn);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

unsigned char processAllMessages() {
	unsigned char messagesLeft = 0;
	tCanMessage msg;
	unsigned long pgn;

	unsigned char messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			pgn = ISO11783Decode(msg.id, 0, 0, 0);
			switch (pgn) {
			case 128259:
				ParsePgn128259(msg.payload, NULL, &waterData.speed.flData);
				waterData.newData = true;
				break;
			case 128267:	
				// Only update the data in waterData if an actual depth was returned. This is only true when it's in the water.
				if (ParsePgn128267(msg.payload, NULL, &waterData.depth.flData, NULL) & 0x02) {
					waterData.newData = true;
				}
				break;
			case 129025:
				ParsePgn129025(msg.payload, &locationData.latitude.flData, &locationData.longitude.flData);
				locationData.newData = true;
				break;
			case 129026:
				ParsePgn129026(msg.payload, NULL, &courseAndSpeedData.courseReference, &courseAndSpeedData.courseOverGround.flData, &courseAndSpeedData.speedOverGround.flData);
				courseAndSpeedData.newData = true;
				break;
			case 129033:
				ParsePgn129033(msg.payload, &timeAndDateData.day, &timeAndDateData.month, &timeAndDateData.year, &timeAndDateData.hour, &timeAndDateData.minute, &timeAndDateData.second);
				timeAndDateData.newData = true;
				break;
			case 130306:
				ParsePgn130306(msg.payload, NULL, &windData.speed.flData, &windData.direction.flData);
				windData.newData = true;
				break;
			case 130310:
				ParsePgn130310(msg.payload, NULL, &waterData.temp.flData, NULL, NULL);
				waterData.newData = true;
				break;
			case 130311:
				ParsePgn130311(msg.payload, NULL, &airData.temp.flData, &airData.humidity.flData, &airData.pressure.flData);
				airData.newData = true;
				break;
			default:
				//DisplayUnhandledId(pgn, msg.payload);
				break;
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
