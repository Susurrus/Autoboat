#include "Node.h"
#include "ecanFunctions.h"
#include "CanMessages.h"

uint16_t status = 0;
uint16_t errors = 0;
uint32_t time = 0;
uint8_t nodeId = 0;
uint8_t cpuLoad = 0;

void NodeTransmitStatus(void)
{
	tCanMessage msg;
	CanMessagePackageStatus(&msg, nodeId, status, errors);
	ecan1_buffered_transmit(&msg);
}