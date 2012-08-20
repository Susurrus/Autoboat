#include "Rudder.h"

struct RudderData rudderDataStore;

void GetRudderData(unsigned char *data) {
	data[0] = rudderDataStore.Position.chData[0];
	data[1] = rudderDataStore.Position.chData[1];
	data[2] = rudderDataStore.StarboardLimit;
	data[3] = rudderDataStore.PortLimit;
}

void SetRudderData(unsigned char *data) {
	rudderDataStore.Position.chData[0] = data[0];
	rudderDataStore.Position.chData[1] = data[1];
	rudderDataStore.StarboardLimit = data[2];
	rudderDataStore.PortLimit = data[3];
}

void ClearRudderData() {
	rudderDataStore.Position.usData = 0;
	rudderDataStore.StarboardLimit = 0;
	rudderDataStore.PortLimit = 0;
}
