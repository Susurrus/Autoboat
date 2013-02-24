#include "Parameters.h"
#include "Node.h"
#include "PrimaryNode.h"
#include "primary_node.h"

#include <stdint.h>
#include <stddef.h>

/**
 * Provides a helper function for retrieving the automode boolean value from the nodeStatus bitfield.
 * @returns A boolean of whether the vehicle is in autonomous mode or not.
 */
uint8_t getAutoMode(void)
{
    return (uint8_t)((nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) > 0);
}

/**
 * Provides a helper function for updating the autonomous mode of the vehicle.
 * @param newMode True to put the vehicle into autonomous mode, False otherwise.
 */
void setAutoMode(uint8_t newMode)
{
    if (newMode) {
        nodeStatus |= PRIMARY_NODE_STATUS_AUTOMODE;
    } else {
        nodeStatus &= ~PRIMARY_NODE_STATUS_AUTOMODE;
    }
}

static Parameter params[] = {
    {"ModeAuto", NULL, (void(*)())setAutoMode, (void(*)())getAutoMode, MAV_PARAM_TYPE_UINT8},
    {"Wheelbase", &wheelbase, NULL, NULL, MAV_PARAM_TYPE_REAL32},
    {"GpsOrigin_Lat", &gpsOrigin[0], NULL, NULL, MAV_PARAM_TYPE_INT32},
    {"GpsOrigin_Lon", &gpsOrigin[1], NULL, NULL, MAV_PARAM_TYPE_INT32},
    {"GpsOrigin_Alt", &gpsOrigin[2], NULL, NULL, MAV_PARAM_TYPE_INT32},
    {"L2+_T*", &tStar, NULL, NULL, MAV_PARAM_TYPE_REAL32},
    {"L2+_IP*", &ipStar, NULL, NULL, MAV_PARAM_TYPE_REAL32},
    {"L2+_InitPoint", &initialPoint, NULL, NULL, MAV_PARAM_TYPE_UINT8},
    {"L2+_T2T", &turn2Track, NULL, NULL, MAV_PARAM_TYPE_UINT8},
    {"L2+_MaxDownPath*", &maxDwnPthStar, NULL, NULL, MAV_PARAM_TYPE_REAL32},
    {"L2+_TanInter", &tanIntercept, NULL, NULL, MAV_PARAM_TYPE_REAL32},
    {"L2+_SwitchDist", &switchDistance, NULL, NULL, MAV_PARAM_TYPE_REAL32}
};

// Expose both the list of parameters and the total to the Parameters library.
Parameter *onboardParameters = params;
const uint16_t PARAMETERS_TOTAL = sizeof(params)/sizeof(Parameter);