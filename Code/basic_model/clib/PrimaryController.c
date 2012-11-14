#include "CanMessages.h"
#include "ecanFunctions.h"
#include "code_gen.h"

void NodeTransmitStatus(void)
{
	tCanMessage msg;
	CanMessagePackageStatus(&msg, CAN_NODE_PRIMARY_CONTROLLER, systemStatus.status, systemStatus.reset);
	ecan1_buffered_transmit(&msg);
}