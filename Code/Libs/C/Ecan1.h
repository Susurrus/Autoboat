/*
 * @file   ecanFunctions.h
 * @author Bryant Mairs
 * @author Pavlo Manovi
 * @date   September 28th, 202
 * @brief  Provides C functions for ECAN blocks
 *
 * This file contains all the functions used by the ecan blocks. It does not
 * contain helper functions such as the circular buffer code.
 *
 */

#ifndef ECAN1_H
#define ECAN1_H

//If simulating, remove the include xc.h  Otherwise, leave it.
#include <xc.h>
#include "EcanDefines.h"
#include "CircularBuffer.h"

#include <stdbool.h>

typedef enum {
    ECAN_ERROR_NONE,
    ECAN_ERROR_WARNING,
    ECAN_ERROR_PASSIVE,
    ECAN_ERROR_BUS_OFF
} EcanError;

typedef struct {
    unsigned TxBufferOverflow: 1; // 1 if a buffer overflow has occured
    unsigned RxBufferOverflow: 1; // 1 if a buffer overflow has occured
    EcanError TxError: 2;
    EcanError RxError: 2;
} EcanStatus;

/**
 * Initialize the CAN hardware. This DOES NOT enable any pins that may be
 * necessary to map as inputs/outputs or using peripheral pin select hardware.
 * Note that DMA0 and DMA2 are utilized for this peripheral.
 * @param f_osc The oscillator frequency of the chip.
 * @param f_baud The desired baud rate of the CAN bus.
 */
void Ecan1Init(uint32_t f_osc, uint32_t f_baud);

/**
 * Pops the top message from the ECAN1 reception buffer.
 * @return A tCanMessage struct with the older message data.
 */
int Ecan1Receive(CanMessage *msg, uint8_t *messagesLeft);

/**
 * Transmits a CAN message via a circular buffer interface
 * similar to that used by CAN message reception.
 */
bool Ecan1Transmit(const CanMessage *message);

/**
 * Returns the error status of the ECAN1 peripheral.
 * Returns an enum
 */
EcanStatus Ecan1GetErrorStatus(void);

/**
 * Returns the transmission and reception error counts. See the documentation for details, but the
 * basics are:
 *   * 1   <= errorCount < 128 : No error
 *   * 128 <= errorCount < 256 : Error
 *   * 256 == errorCount : ECAN bus disabled
 * @param txErrors The transmission error count.
 * @param txErrors The reception error count.
 */
void Ecan1GetErrorCounts(uint8_t *txErrors, uint8_t *rxErrors);

/**
 * This function provides a general way to initialize the DMA peripheral.
 *
 * parameters[0] = IRQ address & squeezed version of DMAxCON minus CHEN bit
 * parameters[1] = address of peripheral (DMAxPAD)
 * parameters[2] = Number of memory units per DMA packet, starting at 1(DMAxCNT)
 * parameters[3] = Primary DPSRAM start address offset bits (DMAxSTA)
 * parameters[4] = Which DMA channel to configure
 * parameters[5] = Secondary DPSRAM start address offset bits (DMAxSTB)
 */
void DmaInit(const uint16_t parameters[6]);

#endif /* ECAN1_H */
