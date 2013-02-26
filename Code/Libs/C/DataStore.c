#include "DataStore.h"
#include "Parameters.h"
#include "DEE.h"
#include "Packing.h"

#include <stdbool.h>

// This indicates the index location, in terms of words, into the EEPROM of the word that tracks
// whether there is good data or not in the EEPROM. All the parameters will be stored in the
// words following this one.
#define HAS_BEEN_WRITTEN_LOCATION 0

// This is the value of the data at HAS_BEEN_WRITTEN_LOCATION if there is good data stored there.
#define GOOD_DATA 1

bool _Write4BytesIntoMemory(const uint8_t data[4], uint8_t *addr)
{
	// And pack those into uint16s.
	uint16_t word1, word2;
	LEUnpackUint16(&word1, &data[0]);
	LEUnpackUint16(&word2, &data[2]);

	// And write into memory (DataEEWrite is 0 on success).
	if (DataEEWrite(word1, *addr)) {
		return false;
	}
	*addr += 1;
	if (DataEEWrite(word2, *addr)) {
		return false;
	}
	*addr += 1;

	// Return success.
	return true;
}

bool _Read4BytesFromMemory(uint8_t data[4], uint8_t *addr)
{
	// Retrieve the 1st word from the memory.
	uint16_t word1 = DataEERead(*addr);
	if (word1 == 0xFFFF) {
		if (dataEEFlags.val) {
			return false;
		}
	}
	*addr += 1;

	// And the 2nd word.
	uint16_t word2 = DataEERead(*addr);
	if (word2 == 0xFFFF) {
		if (dataEEFlags.val) {
			return false;
		}
	}
	*addr += 1;

	// And store them into the provided byte array as little-endian.
	LEPackUint16(&data[0], word1);
	LEPackUint16(&data[2], word2);

	// Return success.
	return true;
}

bool DataStoreStoreAllParameters(void)
{
	// The address to use for this parameter. Stored as some data types are larger than a single
	// word.
	uint8_t offset = HAS_BEEN_WRITTEN_LOCATION + 1;

	int i;
	for (i = 0; i < PARAMETERS_TOTAL; ++i) {
		switch (onboardParameters[i].dataType) {
			case PARAMETERS_DATATYPE_UINT8: {
				// First grab the parameter data.
				uint8_t param;
				ParameterGetValueById(i, &param);

				// And write it into EEPROM (non-zero values mean failure).
				if (DataEEWrite(param, offset)) {
					return false;
				}

				// And increment the memory location for the next parameter.
				++offset;
			} break;
			case PARAMETERS_DATATYPE_UINT32: {
				// First grab the parameter data.
				uint32_t param;
				ParameterGetValueById(i, &param);

				// Now split it up into bytes.
				uint8_t tmp[4];
				LEPackUint32(tmp, param);

				// And write it into EEPROM.
				if (!_Write4BytesIntoMemory(tmp, &offset)) {
					return false;
				}
			} break;
			case PARAMETERS_DATATYPE_INT32: {
				// First grab the parameter data.
				int32_t param;
				ParameterGetValueById(i, &param);

				// Now split it up into bytes.
				uint8_t tmp[4];
				LEPackInt32(tmp, param);

				// And write it into EEPROM.
				if (!_Write4BytesIntoMemory(tmp, &offset)) {
					return false;
				}
			} break;
			case PARAMETERS_DATATYPE_REAL32: {
				// First grab the parameter data.
				float param;
				ParameterGetValueById(i, &param);

				// Now split it up into bytes.
				uint8_t tmp[4];
				LEPackReal32(tmp, param);

				// And write it into EEPROM.
				if (!_Write4BytesIntoMemory(tmp, &offset)) {
					return false;
				}
			} break;
			default:
				return false;
		}
	}

	// Now try to update the metadata and fail out if that can't be updated.
	if (DataEEWrite(GOOD_DATA, HAS_BEEN_WRITTEN_LOCATION) == 0) {
		return true;
	} else {
		return false;
	}
}

bool DataStoreLoadAllParameters(void)
{
	// First check the first memory location. If this value is 1 (instead of the 0xFFFFFFFF the
	// EEPROM on the PICs defaults to), then there is good data stored here. Otherwise, we just
	// fail out as there's no data to load.
	if (DataEERead(HAS_BEEN_WRITTEN_LOCATION) != GOOD_DATA) {
		return false;
	}

	// The address to use for this parameter. Stored as some data types are larger than a single
	// word.
	uint8_t offset = HAS_BEEN_WRITTEN_LOCATION + 1;

	int i;
	for (i = 0; i < PARAMETERS_TOTAL; ++i) {
		switch (onboardParameters[i].dataType) {
			case PARAMETERS_DATATYPE_UINT8:	{
				// Grab the byte from the EEPROM.
				uint8_t param;
				if ((param = DataEERead(offset)) == 0xFFFF) {
					// Error out if an error actually occured. 0xFFFF can be a valid stored value.
					if (dataEEFlags.val) {
						return false;
					}
				}

				// And finally handle writing it into the parameter.
				ParameterSetValueById(i, &param);

				// And increment the memory location for the next parameter.
				++offset;
			} break;
			case PARAMETERS_DATATYPE_UINT32:	{
				// Grab the 4 bytes that make up this parameter from the EEPROM.
				uint8_t tmp[4];
				if (!_Read4BytesFromMemory(tmp, &offset)) {
					return false;
				}

				// And restore it into its proper datatype.
				uint32_t param;
				LEUnpackUint32(&param, tmp);

				// And finally handle writing it into the parameter.
				ParameterSetValueById(i, &param);
			} break;
			case PARAMETERS_DATATYPE_INT32:	{
				// Grab the 4 bytes that make up this parameter from the EEPROM.
				uint8_t tmp[4];
				if (!_Read4BytesFromMemory(tmp, &offset)) {
					return false;
				}

				// And restore it into its proper datatype.
				int32_t param;
				LEUnpackInt32(&param, tmp);

				// And finally handle writing it into the parameter.
				ParameterSetValueById(i, &param);
			} break;
			case PARAMETERS_DATATYPE_REAL32:	{
				// Grab the 4 bytes that make up this parameter from the EEPROM.
				uint8_t tmp[4];
				if (!_Read4BytesFromMemory(tmp, &offset)) {
					return false;
				}

				// And restore it into its proper datatype.
				float param;
				LEUnpackReal32(&param, tmp);

				// And finally handle writing it into the parameter.
				ParameterSetValueById(i, &param);
			} break;
			default:
				return false;
		}
	}

	return true;
}

bool DataStoreInit(void)
{
	// First attempt to initialize the EEPROM unit using Microchip's library.
	if (DataEEInit() != 0) {
		return false;
	}

	// Then attempt to load the onboard parameters. This can fail if:
	//  a) There was a problem with the EEPROM OR
	//  b) There are no saved onboard parameters.
    // In that case we save the current values and load them again and make sure everything worked.

	// And load all stored parameters in the EEPROM. If this errors out, assume its because the
	// EEPROM is currently empty (like if the PIC was just flashed). So write the current parameters
	// and try reading them again and only error out if either of those fail.
	if (!DataStoreLoadAllParameters()) {
		if (DataStoreStoreAllParameters()) {
			if (!DataStoreLoadAllParameters()) {
				return false;
			}
		} else {
			return false;
		}
	}

	return true;
}