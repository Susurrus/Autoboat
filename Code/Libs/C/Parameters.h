#ifndef PARAMETERS_H
#define PARAMETERS_H

/**
 * @file
 * @brief This implements a simple dictionary-like interface to onboard parameters.
 *
 * # Dependencies
 *  * stdint.h - Possibly a C99 implementation, though most compilers support this now.
 *
 * # Usage
 * Implement a corresponding ParametersHelper.c file that defines both the PARAMETERS_TOTAL constant
 * as well as the onboardParameters array. The onboardParameters array will be an array of Parameter
 * structs. Note that all variables should either have a direct data pointer xor setter/getter
 * functions. For read-only parameters, not defining a Setter function is acceptable.
 */
#include <stdint.h>

/**
 * Declare the various datatypes supported by the parameter interface. UINT refers to unsigned
 * integers, INT to signed integers, REAL to floating point numbers. The numbers following refer to
 * the size of the values in bits. REAL32/REAL64 map to the C float and double datatypes
 * respectively and may have additional limitations because of that.
 * Also note that the PARAMETERS_DATATPYE enum follows from the MAVLink MAV_PARAM enum, such that
 * MAVLink PARAM_* packets can interface directly with this code.
 */
enum PARAMETERS_DATATYPE {
	PARAMETERS_DATATYPE_UINT8 = 1,
	PARAMETERS_DATATYPE_INT8 = 2,
	PARAMETERS_DATATYPE_UINT16 = 3,
	PARAMETERS_DATATYPE_INT16 = 4,
	PARAMETERS_DATATYPE_UINT32 = 5,
	PARAMETERS_DATATYPE_INT32 = 6,
	PARAMETERS_DATATYPE_UINT64 = 7,
	PARAMETERS_DATATYPE_INT64 = 8,
	PARAMETERS_DATATYPE_REAL32 = 9,
	PARAMETERS_DATATYPE_REAL64 = 10
};

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
    const enum PARAMETERS_DATATYPE dataType;
} Parameter;

// The total number of parameters onboard. This should be set by external code written by the user
// of the library. Specifies the size of `onboardParameters`.
extern const uint16_t PARAMETERS_TOTAL;

// The array of parameters stored onboard. This should be set by external code written by the user
// of the library. Should have as many elements as PARAMETERS_TOTAL.
extern Parameter *onboardParameters;

/**
 * Given a parameter name and a new value, update that specific parameter value.
 * @param name[in] The name of the parameter as a NULL-terminated C string.
 * @param value[in] A pointer to the new data for the value.
 * @return The corresponding integer index of the set variable. Will be UINT16_MAX on error.
 */
uint16_t ParameterSetValueByName(const char *name, const void *value);

/**
 * Given a parameter ID and a value, update that specific parameter. This function either writes the
 * value directly if a memory address is specified or calls the proper Setter() function.
 * @param id The ID of the parameter.
 * @param value A pointer to the data to set this parameter to.
 */
void ParameterSetValueById(uint16_t id, const void *value);

/**
 * Retrieves the current value of a parameter by name. For accessing the parameter by index, use the
 * onboardParameters array directly.
 * @param name[in] The name of the parameter as a NULL-terminated C string.
 * @param value[out] The location to store the data.
 */
uint16_t ParameterGetValueByName(const char *name, void *value);

/**
 * Given a parameter ID and a value, retrieve that specific parameter's current value. This function
 * either reads the value directly if a memory address is specified or calls the proper Getter()
 * function.
 * @param id The ID of the parameter.
 * @param value A pointer to the data to write this parameter's value into.
 */
void ParameterGetValueById(uint16_t id, void *value);

#endif // PARAMETERS_H
