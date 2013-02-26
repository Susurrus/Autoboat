#ifndef DATA_STORE_H
#define DATA_STORE_H

/**
 * @file
 * @brief A EEPROM backend for the Parameters library.
 *
 * # Dependencies
 *  * DEE library from Microchip (DEE.h, DEE.c, DEE.s)
 *  * Parameter library.
 *  * Packing library.
 *
 * # Usage
 * This DataStore library implements an EEPROM backend for the Parameters library. Any values used
 * by the library (so declared as an array of `Parameter` structs and pointed to by the
 * `onboardParameters` variable) are automatically used by this library. Call `DataStoreInit()`
 * first and then used DataStoreLoadAllParameters() to load parameters from the EEPROM into the
 * parameter data locations and DataStoreStoreAllParameters() to save the current parameters into
 * the EEPROM. So simple, you won't believe!
 *
 * NOTE: Only the datatypes UINT8, UINT16, UINT32, INT32, and REAL32 have been implemented as of
 * right now!
 */

#include <stdbool.h>

/**
 * An enum for the return values of DataStoreInit(). Specifically it allows for distinguishing
 * between Init() calls where there was valid data in the EEPROM and when there wasn't.
 */
enum DATASTORE_INIT {
	DATASTORE_INIT_FAIL = 0, // Failed.
	DATASTORE_INIT_SUCCESS,  // Succeeded with no qualifiers.
	DATASTORE_INIT_PRELOADED // Succeeded, but the EEPROM was empty and was preloaded with current parameter data.
};

/**
 * Initializes the underlying storage medium (the EEPROM as of right now) and also makes sure that a
 * copy of the onboard parameters have been written to permanent storage. This ensures that all
 * subsequent calls to DataStoreLoadAllParameters() will fail only due to EEPROM issues.
 * @return true if it succeeded, false if it didn't. Probably a critical error.
 */
enum DATASTORE_INIT DataStoreInit(void);

/**
 * Retrieves the values of all parameters stored in the EEPROM and loads them into the global
 * variables as referenced by the `onboardParameters` array used with the Parameters library. Note
 * that this function can fail if there's an error with the EEPROM OR if there are no parametrs
 * stored in the EEPROM. This shouldn't be an issue if it's probably used after DataStoreInit() is
 * called as that function makes sure that there's data stored in the EEPROM.
 * @return true if loading succeeded and parameters were updated, false otherwise.
 */
bool DataStoreSaveParameters(void);

/**
 * This function stores all parameters that are exposed to the Parameters library to EEPROM. The
 * situation where the memory has never been written, such as right after a flashing and on first
 * boot-up, is handled gracefully where no values are loaded and a false is returned. There is no
 * way to segregate between failure and this situation.
 * @return true if the function succeeded, false if there was a problem restoring values from EEPROM.
 */
bool DataStoreLoadParameters(void);

#endif // DATA_STORE_H