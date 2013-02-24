#include "Parameters.h"

#include <stdint.h>

/**
 * Defines all of the work necessary for updating a parameter based on the onboardParameters array.
 * @param type A valid C datatype.
 */
#define SET_PARAM(type) \
if (onboardParameters[id].data) { \
	*(type*)onboardParameters[id].data = *(type*)value; \
} else if (onboardParameters[id].Setter) { \
	void (*SetParam)(type) = (void(*)(type))onboardParameters[id].Setter; \
	SetParam(*(type*)value); \
}

/**
 * Encapsulates everything necessary for retrieving a parameter based on the data in
 * onboardParameters.
 * @param type A valid C datatype.
 */
#define GET_PARAM(type) \
if (onboardParameters[id].data) { \
	*(type*)value = *(type*)onboardParameters[id].data; \
} else if (onboardParameters[id].Getter) { \
	type (*GetParam)(void) = (type(*)(void))onboardParameters[id].Getter; \
	*(type*)value = GetParam(); \
}

/**
 * Given a parameter name and a new value, update that that specific parameter value.
 * @param id[in] The id of the parameter to set.
 * @param value[in] A pointer to the new data for the value.
 * @return The corresponding integer index of the set variable. Will be UINT16_MAX on error.
 */
void ParameterSetValueById(uint16_t id, const void *value)
{
	if (id < PARAMETERS_TOTAL) {
		switch(onboardParameters[id].dataType) {
			case MAV_PARAM_TYPE_UINT8:
				SET_PARAM(uint8_t);
				break;
			case MAV_PARAM_TYPE_INT8:
				SET_PARAM(int8_t);
				break;
			case MAV_PARAM_TYPE_UINT16:
				SET_PARAM(uint16_t);
				break;
			case MAV_PARAM_TYPE_INT16:
				SET_PARAM(int16_t);
				break;
			case MAV_PARAM_TYPE_UINT32:
				SET_PARAM(uint32_t);
				break;
			case MAV_PARAM_TYPE_INT32:
				SET_PARAM(int32_t);
				break;
			case MAV_PARAM_TYPE_UINT64:
				SET_PARAM(uint64_t);
				break;
			case MAV_PARAM_TYPE_INT64:
				SET_PARAM(int64_t);
				break;
			case MAV_PARAM_TYPE_REAL32:
				SET_PARAM(float);
				break;
			case MAV_PARAM_TYPE_REAL64:
				SET_PARAM(double);
				break;
			case MAV_PARAM_TYPE_ENUM_END:
			default:
				break;
		}
	}
}

/**
 * Given a parameter name and a new value, update that that specific parameter value.
 * @param name[in] The name of the parameter as a NULL-terminated C string.
 * @param value[in] A pointer to the new data for the value.
 * @return The corresponding integer index of the set variable. Will be UINT16_MAX on error.
 */
uint16_t ParameterSetValueByName(const char *name, const void *value)
{
	uint16_t i;
	for (i = 0; i < PARAMETERS_TOTAL; ++i) {
		if (strcmp(name, onboardParameters[i].name) == 0) {
			ParameterSetValueById(i, value);
			return i;
		}
	}

	// If we couldn't do anything, return our error code.
	return UINT16_MAX;
}

/**
 * Retrieves the current value of a parameter by name. For accessing the parameter by index, use the
 * onboardParameters array directly.
 * @param id[in] The id of the parameter.
 * @param value[out] The location to store the data.
 */
void ParameterGetValueById(uint16_t id, void *value)
{
	if (id < PARAMETERS_TOTAL) {
		switch(onboardParameters[id].dataType) {
			case MAV_PARAM_TYPE_UINT8:
				GET_PARAM(uint8_t);
				break;
			case MAV_PARAM_TYPE_INT8:
				GET_PARAM(int8_t);
				break;
			case MAV_PARAM_TYPE_UINT16:
				GET_PARAM(uint16_t);
				break;
			case MAV_PARAM_TYPE_INT16:
				GET_PARAM(int16_t);
				break;
			case MAV_PARAM_TYPE_UINT32:
				GET_PARAM(uint32_t);
				break;
			case MAV_PARAM_TYPE_INT32:
				GET_PARAM(int32_t);
				break;
			case MAV_PARAM_TYPE_UINT64:
				GET_PARAM(uint64_t);
				break;
			case MAV_PARAM_TYPE_INT64:
				GET_PARAM(int64_t);
				break;
			case MAV_PARAM_TYPE_REAL32:
				GET_PARAM(float);
				break;
			case MAV_PARAM_TYPE_REAL64:
				GET_PARAM(double);
				break;
			case MAV_PARAM_TYPE_ENUM_END:
			default:
				break;
		}
	}
}

/**
 * Retrieves the current value of a parameter by name. For accessing the parameter by index, use the
 * onboardParameters array directly.
 * @param name[in] The name of the parameter as a NULL-terminated C string.
 * @param value[out] The location to store the data.
 */
uint16_t ParameterGetValueByName(const char *name, void *value)
{
	uint16_t i;
	for (i = 0; i < PARAMETERS_TOTAL; ++i) {
		if (strcmp(name, onboardParameters[i].name) == 0) {
			ParameterGetValueById(i, value);
			return i;
		}
	}

	// Otherwise return an error code.
	return UINT16_MAX;
}