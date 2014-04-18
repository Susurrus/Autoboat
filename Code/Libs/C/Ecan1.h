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
void Ecan1Transmit(const CanMessage *message);

/**
 * Returns the error status of the ECAN1 peripheral.
 * Returns a tuple with element 0->transmission error state,
 * and element 1->reception error state.
 *  0 => no error
 *  1 => warning (error count E(96,128]
 *  2 => passive (error count E(128,256] 
 *  3 => off (error count > 256, only for TX)
 */
void Ecan1GetErrorStatus(uint8_t errors[2]);

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
