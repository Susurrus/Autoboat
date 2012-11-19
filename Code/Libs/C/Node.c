#include "Node.h"
#include "ecanFunctions.h"
#include "CanMessages.h"

uint16_t status;
uint16_t errors;
uint32_t time;
uint8_t nodeId;

void NodeTransmitStatus(void)
{
	tCanMessage msg;
	CanMessagePackageStatus(&msg, CAN_NODE_PRIMARY_CONTROLLER, status, errors);
	ecan1_buffered_transmit(&msg);
}