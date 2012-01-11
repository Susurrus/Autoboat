#include "Rudder.h"
#include "types.h"

// Declaration of the relevant message structs used.
static struct {
	tUnsignedShortToChar Position;
	unsigned char PortLimit;
	unsigned char StarboardLimit;
} tRudderData;

void GetRudderData(unsigned char *data) {
	data[0] = tRudderData.Position.chData[0];
	data[1] = tRudderData.Position.chData[1];
	data[2] = tRudderData.StarboardLimit;
	data[3] = tRudderData.PortLimit;
}

void SetRudderData(unsigned char *data) {
	tRudderData.Position.chData[0] = data[0];
	tRudderData.Position.chData[1] = data[1];
	tRudderData.StarboardLimit = data[2];
	tRudderData.PortLimit = data[3];
}

void ClearRudderData() {
	tRudderData.Position.usData = 0;
	tRudderData.StarboardLimit = 0;
	tRudderData.PortLimit = 0;
}

