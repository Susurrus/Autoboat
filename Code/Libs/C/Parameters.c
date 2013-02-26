#include "Parameters.h"

#include <stdint.h>
#include <string.h>

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

void ParameterSetValueById(uint16_t id, const void *value)
{
	if (id < PARAMETERS_TOTAL) {
		switch(onboardParameters[id].dataType) {
			case PARAMETERS_DATATYPE_UINT8:
				SET_PARAM(uint8_t);
				break;
			case PARAMETERS_DATATYPE_INT8:
				SET_PARAM(int8_t);
				break;
			case PARAMETERS_DATATYPE_UINT16:
				SET_PARAM(uint16_t);
				break;
			case PARAMETERS_DATATYPE_INT16:
				SET_PARAM(int16_t);
				break;
			case PARAMETERS_DATATYPE_UINT32:
				SET_PARAM(uint32_t);
				break;
			case PARAMETERS_DATATYPE_INT32:
				SET_PARAM(int32_t);
				break;
			case PARAMETERS_DATATYPE_UINT64:
				SET_PARAM(uint64_t);
				break;
			case PARAMETERS_DATATYPE_INT64:
				SET_PARAM(int64_t);
				break;
			case PARAMETERS_DATATYPE_REAL32:
				SET_PARAM(float);
				break;
			case PARAMETERS_DATATYPE_REAL64:
				SET_PARAM(double);
				break;
			default:
				break;
		}
	}
}

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

void ParameterGetValueById(uint16_t id, void *value)
{
	if (id < PARAMETERS_TOTAL) {
		switch(onboardParameters[id].dataType) {
			case PARAMETERS_DATATYPE_UINT8:
				GET_PARAM(uint8_t);
				break;
			case PARAMETERS_DATATYPE_INT8:
				GET_PARAM(int8_t);
				break;
			case PARAMETERS_DATATYPE_UINT16:
				GET_PARAM(uint16_t);
				break;
			case PARAMETERS_DATATYPE_INT16:
				GET_PARAM(int16_t);
				break;
			case PARAMETERS_DATATYPE_UINT32:
				GET_PARAM(uint32_t);
				break;
			case PARAMETERS_DATATYPE_INT32:
				GET_PARAM(int32_t);
				break;
			case PARAMETERS_DATATYPE_UINT64:
				GET_PARAM(uint64_t);
				break;
			case PARAMETERS_DATATYPE_INT64:
				GET_PARAM(int64_t);
				break;
			case PARAMETERS_DATATYPE_REAL32:
				GET_PARAM(float);
				break;
			case PARAMETERS_DATATYPE_REAL64:
				GET_PARAM(double);
				break;
			default:
				break;
		}
	}
}

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