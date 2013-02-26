#include "Parameters.h"
#include "RcNode.h"

#include <stddef.h>

static const Parameter params[] = {
    {"Rudder_Low", &rcRudderRange[0], NULL, NULL, PARAMETERS_DATATYPE_UINT16},
    {"Rudder_High", &rcRudderRange[1], NULL, NULL, PARAMETERS_DATATYPE_UINT16},
    {"Throttle_Low", &rcThrottleRange[0], NULL, NULL, PARAMETERS_DATATYPE_UINT16},
    {"Throttle_High", &rcThrottleRange[1], NULL, NULL, PARAMETERS_DATATYPE_UINT16}
};

// Expose both the list of parameters and the total to the Parameters library.
const Parameter *onboardParameters = params;
const uint16_t PARAMETERS_TOTAL = sizeof(params)/sizeof(Parameter);