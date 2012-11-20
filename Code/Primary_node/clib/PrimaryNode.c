#include "Node.h"
#include "CommProtocol.h"
#include "MavlinkGlue.h"
#include "DEE.h"

void PrimaryNodeInit(void)
{
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;
	cpInitCommunications();
	DataEEInitAndClear();
	MavLinkInit();
}