#include "Ecan1.h"

/**
 * Returns a 4-byte array with details on any ECAN errors.
 * Required because of the limitations in the MATLAB c-function
 * call block.
 * @return TX error, RX error 
 * @see Ecan1.h
 */
void Ecan1ErrorsMatlab(uint8_t errors[2])
{
    EcanStatus x = Ecan1GetErrorStatus();
    errors[0] = x.TxError;
    errors[1] = x.RxError;
}