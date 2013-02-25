#ifndef DATA_STORE_H
#define DATA_STORE_H

#include <stdbool.h>

bool DataStoreInit(void);
bool DataStoreLoadAllParameters(void);

/**
 * This function stores all parameters that are exposed to the Parameters library to EEPROM. The
 * situation where the memory has never been written, such as right after a flashing and on first
 * boot-up, is handled gracefully where no values are loaded and a false is returned. There is no
 * way to segregate between failure and this situation.
 * @return true if the function succeeded, false if there was a problem restoring values from EEPROM.
 */
bool DataStoreStoreAllParameters(void);

#endif // DATA_STORE_H