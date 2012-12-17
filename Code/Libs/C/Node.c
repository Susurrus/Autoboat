#include "Node.h"
#include "Ecan1.h"
#include "CanMessages.h"

uint8_t nodeId = 0;
uint8_t nodeCpuLoad = 0;
uint16_t nodeStatus = 0;
uint16_t nodeErrors = 0;
uint32_t nodeSystemTime = 0;

void NodeTransmitStatus(void)
{
	CanMessage msg;
	CanMessagePackageStatus(&msg, nodeId, nodeStatus, nodeErrors, nodeCpuLoad);
	Ecan1BufferedTransmit(&msg);
}