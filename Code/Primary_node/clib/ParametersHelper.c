#include "Parameters.h"
#include "Node.h"
#include "PrimaryNode.h"
#include "primary_node.h"
#include "MavlinkGlue.h"

#include <stdint.h>
#include <stddef.h>

/**
 * Provides a helper function for retrieving the automode boolean value from the nodeStatus bitfield.
 * @returns A boolean of whether the vehicle is in autonomous mode or not.
 */
uint8_t GetAutoMode(void)
{
    return (uint8_t)((nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) > 0);
}

/**
 * Provides a helper function for updating the autonomous mode of the vehicle. When the vehicle is
 * switched into autonomous mode, a new starting location is stored.
 * @param newMode True to put the vehicle into autonomous mode, False otherwise.
 */
void SetAutoMode(uint8_t newMode)
{
    if (newMode) {
		// Store the current vehicle position when autonomous control is enabled.
		if ((nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) == 0) {
			SetStartingPointToCurrentLocation();
		}
        nodeStatus |= PRIMARY_NODE_STATUS_AUTOMODE;
    } else {
        nodeStatus &= ~PRIMARY_NODE_STATUS_AUTOMODE;
    }
}

void ToggleAutoMode(void)
{
	SetAutoMode(!GetAutoMode());
}

static const Parameter params[] = {
    {"ModeAuto", NULL, (void(*)())SetAutoMode, (void(*)())GetAutoMode, PARAMETERS_DATATYPE_UINT8},
    {"Wheelbase", &wheelbase, NULL, NULL, PARAMETERS_DATATYPE_REAL32},
    {"Gps_SlewLimit", &gps_leap_rate_limit, NULL, NULL, PARAMETERS_DATATYPE_INT32},
    {"Gps_OriginLat", &gpsOrigin[0], NULL, NULL, PARAMETERS_DATATYPE_INT32},
    {"Gps_OriginLon", &gpsOrigin[1], NULL, NULL, PARAMETERS_DATATYPE_INT32},
    {"Gps_OriginAlt", &gpsOrigin[2], NULL, NULL, PARAMETERS_DATATYPE_INT32},
    {"L2+_T*", &tStar, NULL, NULL, PARAMETERS_DATATYPE_REAL32},
    {"L2+_IP*", &ipStar, NULL, NULL, PARAMETERS_DATATYPE_REAL32},
    {"L2+_InitPoint", &initialPoint, NULL, NULL, PARAMETERS_DATATYPE_UINT8},
    {"L2+_T2T", &turn2Track, NULL, NULL, PARAMETERS_DATATYPE_UINT8},
    {"L2+_MaxDownPath*", &maxDwnPthStar, NULL, NULL, PARAMETERS_DATATYPE_REAL32},
    {"L2+_TanInter", &tanIntercept, NULL, NULL, PARAMETERS_DATATYPE_REAL32},
    {"L2+_SwitchDist", &switchDistance, NULL, NULL, PARAMETERS_DATATYPE_REAL32}
};

// Expose both the list of parameters and the total to the Parameters library.
const Parameter *onboardParameters = params;
const uint16_t PARAMETERS_TOTAL = sizeof(params)/sizeof(Parameter);