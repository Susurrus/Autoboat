#include "Node.h"
#include "Hil.h"

void HilNodeInit(void)
{
	// Set a unique node ID for this node.
	nodeId = CAN_NODE_HIL;
	
	// Initialize communications for HIL.
	HilInit();
}
