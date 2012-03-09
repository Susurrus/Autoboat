#include "ecanSensors.h"

#include "ecanDefinitions.h"
#include "nmea2000.h"
#include "types.h"
#include "code_gen.h"

struct PowerData powerDataStore;
struct WindData windDataStore;
struct AirData airDataStore;
struct WaterData waterDataStore;
struct ThrottleData throttleDataStore;
struct GpsData gpsDataStore;
struct DateTimeData dateTimeDataStore;
uint8_t sensorsEnabled = 0; // Initially assume that nothing's enabled.
uint8_t sensorsHealthy = 0; // Initially assume that nothing's active.

void GetWindDataPacked(unsigned char *data)
{
	data[0] = windDataStore.speed.chData[0];
	data[1] = windDataStore.speed.chData[1];
	data[2] = windDataStore.speed.chData[2];
	data[3] = windDataStore.speed.chData[3];
	data[4] = windDataStore.direction.chData[0];
	data[5] = windDataStore.direction.chData[1];
	data[6] = windDataStore.direction.chData[2];
	data[7] = windDataStore.direction.chData[3];
	data[8] = windDataStore.newData;
	windDataStore.newData = false;
}

void GetAirDataPacked(unsigned char *data)
{
	data[0] = airDataStore.temp.chData[0];
	data[1] = airDataStore.temp.chData[1];
	data[2] = airDataStore.temp.chData[2];
	data[3] = airDataStore.temp.chData[3];
	data[4] = airDataStore.pressure.chData[0];
	data[5] = airDataStore.pressure.chData[1];
	data[6] = airDataStore.pressure.chData[2];
	data[7] = airDataStore.pressure.chData[3];
	data[8] = airDataStore.humidity.chData[0];
	data[9] = airDataStore.humidity.chData[1];
	data[10] = airDataStore.humidity.chData[2];
	data[11] = airDataStore.humidity.chData[3];
	data[12] = airDataStore.newData;
	airDataStore.newData = false;
}

void GetWaterDataPacked(unsigned char *data)
{
	data[0] = waterDataStore.speed.chData[0];
	data[1] = waterDataStore.speed.chData[1];
	data[2] = waterDataStore.speed.chData[2];
	data[3] = waterDataStore.speed.chData[3];
	data[4] = waterDataStore.temp.chData[0];
	data[5] = waterDataStore.temp.chData[1];
	data[6] = waterDataStore.temp.chData[2];
	data[7] = waterDataStore.temp.chData[3];
	data[8] = waterDataStore.depth.chData[0];
	data[9] = waterDataStore.depth.chData[1];
	data[10] = waterDataStore.depth.chData[2];
	data[11] = waterDataStore.depth.chData[3];
	data[12] = waterDataStore.newData;
	waterDataStore.newData = false;
}

void GetThrottleDataPacked(unsigned char *data)
{
	data[0] = throttleDataStore.rpm.chData[0];
	data[1] = throttleDataStore.rpm.chData[1];
	data[2] = throttleDataStore.newData;
	throttleDataStore.newData = false;
}

void GetGpsDataPacked(unsigned char* data)
{
	data[0] = gpsDataStore.lat.chData[0];
	data[1] = gpsDataStore.lat.chData[1];
	data[2] = gpsDataStore.lat.chData[2];
	data[3] = gpsDataStore.lat.chData[3];
	data[4] = gpsDataStore.lon.chData[0];
	data[5] = gpsDataStore.lon.chData[1];
	data[6] = gpsDataStore.lon.chData[2];
	data[7] = gpsDataStore.lon.chData[3];
	data[8] = gpsDataStore.alt.chData[0];
	data[9] = gpsDataStore.alt.chData[1];
	data[10] = gpsDataStore.alt.chData[2];
	data[11] = gpsDataStore.alt.chData[3];
	
	data[12] = gpsDataStore.cog.chData[0];
	data[13] = gpsDataStore.cog.chData[1];
	data[14] = gpsDataStore.cog.chData[2];
	data[15] = gpsDataStore.cog.chData[3];
	data[16] = gpsDataStore.sog.chData[0];
	data[17] = gpsDataStore.sog.chData[1];
	data[18] = gpsDataStore.sog.chData[2];
	data[19] = gpsDataStore.sog.chData[3];
	
	data[20] = gpsDataStore.newData;
	
	// Mark this data as old now
	gpsDataStore.newData = 0;
}

void ClearGpsData(void)
{
	gpsDataStore.lat.flData = 0.0;
	gpsDataStore.lon.flData = 0.0;
	gpsDataStore.alt.flData = 0.0;
	gpsDataStore.cog.flData = 0.0;
	gpsDataStore.sog.flData = 0.0;
	gpsDataStore.newData = 0;
}

unsigned char ProcessAllEcanMessages()
{
	uint8_t messagesLeft = 0;
	tCanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == 0x402) {
				throttleDataStore.rpm.shData = (int)(((unsigned int)msg.payload[0]) << 8) | ((unsigned int)msg.payload[1]);
				throttleDataStore.newData = true;
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case 126992: {
					uint8_t rv = ParsePgn126992(msg.payload, NULL, NULL, &dateTimeDataStore.year, &dateTimeDataStore.month, &dateTimeDataStore.day, &dateTimeDataStore.hour, &dateTimeDataStore.min, &dateTimeDataStore.sec);
					// Check if all 6 parts of the datetime were successfully decoded before triggering an update
					if (rv & 0xFC == 0xFC) {
						dateTimeDataStore.newData = true;
					}
				} break;
				case 127508: {
					uint8_t rv = ParsePgn127508(msg.payload, NULL, NULL, &powerDataStore.voltage.flData, &powerDataStore.current.flData, &powerDataStore.temperature.flData);
					if ((rv & 0x0C) == 0xC) {
						powerDataStore.newData = true;
					}
				} break;
				case 128259:
					if (ParsePgn128259(msg.payload, NULL, &waterDataStore.speed.flData)) {
						waterDataStore.newData = true;
					}
				break;
				case 128267: {
					// Only update the data in waterDataStore if an actual depth was returned.
					uint8_t rv = ParsePgn128267(msg.payload, NULL, &waterDataStore.depth.flData, NULL);
					if ((rv & 0x02) == 0x02) {
						waterDataStore.newData = true;
					}
				} break;
				case 129025: {
					// Only record the live GPS data if we aren't in HIL mode.
					if ((systemStatus.status & (1 << 1)) == 0) {
						uint8_t rv = ParsePgn129025(msg.payload, &gpsDataStore.lat.flData, &gpsDataStore.lon.flData);
						// Only update if both latitude and longitude were parsed successfully.
						if ((rv & 0x03) == 0x03) {
							gpsDataStore.newData = true;
						}
					}
				} break;
				case 129026: {
					// Only record the live GPS data if we aren't in HIL mode.
					if ((systemStatus.status & (1 << 1)) == 0) {
						uint8_t rv = ParsePgn129026(msg.payload, NULL, NULL, &gpsDataStore.cog.flData, &gpsDataStore.sog.flData);
						// Only update if both course-over-ground and speed-over-ground were parsed successfully.
						if ((rv & 0x0C) == 0x0C) {
							gpsDataStore.newData = true;
						}
					}
				} break;
				case 130306:
					if (ParsePgn130306(msg.payload, NULL, &windDataStore.speed.flData, &windDataStore.direction.flData)) {
						windDataStore.newData = true;
					}
				break;
				case 130310:
					if (ParsePgn130310(msg.payload, NULL, &waterDataStore.temp.flData, NULL, NULL)) {
						waterDataStore.newData = true;
					}
				break;
				case 130311:
					if (ParsePgn130311(msg.payload, NULL, NULL, NULL, &airDataStore.temp.flData, &airDataStore.humidity.flData, &airDataStore.pressure.flData)) {
						airDataStore.newData = true;
					}
				break;
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
