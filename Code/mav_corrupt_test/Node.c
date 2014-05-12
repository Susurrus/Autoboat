#include "Node.h"

// Declare all of the variables here. Also set all of them to an invalid value. This makes it easier
// if these values are transmit and they aren't used as they'll already be set to invalid.
uint8_t nodeId = UINT8_MAX;
uint8_t nodeCpuLoad = UINT8_MAX;
int8_t nodeTemp = INT8_MAX;
uint8_t nodeVoltage = UINT8_MAX;
uint16_t nodeStatus = 0;
uint16_t nodeErrors = 0;
uint32_t nodeSystemTime = 0;

void NodeTransmitStatus(void)
{
}