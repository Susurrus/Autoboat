#include "Parameters.h"
#include "RudderNode.h"

#include <stddef.h>

static const Parameter params[] = {
    {"Limit_Port", &rudderCalData.PortLimitValue, NULL, NULL, PARAMETERS_DATATYPE_UINT16},
    {"Limit_SB", &rudderCalData.StarLimitValue, NULL, NULL, PARAMETERS_DATATYPE_UINT16}
};

// Expose both the list of parameters and the total to the Parameters library.
const Parameter *onboardParameters = params;
const uint16_t PARAMETERS_TOTAL = sizeof(params)/sizeof(Parameter);
