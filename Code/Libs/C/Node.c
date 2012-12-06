#include "Node.h"
#include "ecanFunctions.h"
#include "CanMessages.h"

uint8_t nodeId = 0;
uint8_t nodeCpuLoad = 0;
uint16_t nodeStatus = 0;
uint16_t nodeErrors = 0;
uint32_t nodeSystemTime = 0;

void NodeTransmitStatus(void)
{
	tCanMessage msg;
	CanMessagePackageStatus(&msg, nodeId, status, errors);
	ecan1_buffered_transmit(&msg);
}