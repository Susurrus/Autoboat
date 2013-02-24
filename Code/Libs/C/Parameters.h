#ifndef PARAMETERS_H
#define PARAMETERS_H

/**
 * @file
 * @brief This implements a simple dictionary-like interface to onboard parameters.
 *
 * # Usage
 * Implement a corresponding Parameters.c file that defines both the PARAMETERS_TOTAL constant as
 * well as the onboardParameters array. The onboardParameters array will be an array of Parameter
 * structs. Note that all variables should either have a direct data pointer or setter/getter
 * functions. For read-only parameters, not defining a Setter function is acceptable.
 */
#include "mavlink.h"
#include <stdint.h>

/**
 * Declare a struct storing all necessary information for a parameter.
 * Note that data OR setter/getter should be set for variables' data indicates
 * that the data pointer can be directly accessed. Otherwise the setter/getter
 * functions should be used instead. Setter functions should look like `void F(DATATYPE)` while
 * getter functions should look like `DATATYPE F(void)`.
 */
typedef struct Parameter {
    const char name[16];
    void *data;
    void (*Setter)(void);
    void (*Getter)(void);
    const enum MAV_PARAM_TYPE dataType;
} Parameter;

// The total number of parameters onboard
extern const uint16_t PARAMETERS_TOTAL;

// The array of parameters stored onboard.
extern Parameter *onboardParameters;

void ParameterSetValueById(uint16_t id, const void *value);

uint16_t ParameterSetValueByName(const char *name, const void *value);

void ParameterGetValueById(uint16_t id, void *value);

uint16_t ParameterGetValueByName(const char *name, void *value);

#endif // PARAMETERS_H
