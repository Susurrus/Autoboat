// Include custom library headers
#include "Ecan1.h"
#include "CircularBuffer.h"

// Include standard C library headers
#include <string.h>
#include <stdbool.h>

// Include Microchip library headers
#include <ecan.h>
#include <dma.h>

/**
 * @file   Ecan1.c
 * @author Bryant Mairs
 * @author Pavlo Manovi
 * @date   September 28th, 2012
 * @brief  Provides C functions for ECAN blocks
 */

// Specify the number of 8-byte CAN messages buffer supports.
// This can be overridden by user code.
#ifndef ECAN1_BUFFERSIZE
#define ECAN1_BUFFERSIZE 8 * 24
#endif

// Declare space for our message buffer in DMA
// NOTE: This DMA space is aligned along 64-byte boundaries to make sure there's enough room for
// all 64-bytes of memory required.
#ifdef __dsPIC33FJ128MC802__
static volatile uint16_t ecan1MsgBuf[4][8] __attribute__((space(dma), aligned(64)));
#elif __dsPIC33EP256MC502__
static volatile uint16_t ecan1MsgBuf[4][8] __attribute__((aligned(64)));
#endif

// Initialize our circular buffers and data arrays for transreceiving CAN messages
static CircularBuffer ecan1RxCBuffer;
static uint8_t rxDataArray[ECAN1_BUFFERSIZE];
static CircularBuffer ecan1TxCBuffer;
static uint8_t txDataArray[ECAN1_BUFFERSIZE];

// Track whether or not we're currently transmitting
static bool currentlyTransmitting = 0;

// Also track how many messages are pending for reading.
static uint8_t receivedMessagesPending = 0;

void Ecan1Init(uint32_t f_osc, uint32_t f_baud)
{
    // Initialize our circular buffers. If this fails, we crash and burn.
    if (!CB_Init(&ecan1TxCBuffer, txDataArray, ECAN1_BUFFERSIZE)) {
        while (1);
    }
    if (!CB_Init(&ecan1RxCBuffer, rxDataArray, ECAN1_BUFFERSIZE)) {
        while (1);
    }

    // Set ECAN1 into configuration mode and wait until it switches modes.
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    // Initialize the CAN node. We assume a fixed length for all of the CAN
    // segments, but otherwise let the user specify the baud rate.
    const uint16_t propagationSegmentLength = 5;
    const uint16_t phaseSegment1Length = 8;
    const uint16_t phaseSegment2Length = 6;
    const uint64_t f_tq = (propagationSegmentLength + phaseSegment1Length + phaseSegment2Length + 1) * f_baud;
    const uint64_t f_cy = f_osc / 2;
    const uint8_t brp = (uint8_t)(f_cy / (2L * f_tq));

    CAN1Initialize(CAN_SYNC_JUMP_WIDTH4 & CAN_BAUD_PRE_SCALE(brp),
            CAN_WAKEUP_BY_FILTER_DIS & CAN_PROPAGATIONTIME_SEG_TQ(propagationSegmentLength) & CAN_PHASE_SEG1_TQ(phaseSegment1Length) & CAN_PHASE_SEG2_TQ(phaseSegment2Length) & CAN_SEG2_FREE_PROG & CAN_SAMPLE3TIMES);

    // Use 4 buffers in DMA RAM (smallest value we can choose), other option is irrelevant.
    CAN1FIFOCon(CAN_DMA_BUF_SIZE_4 & CAN_FIFO_AREA_TRB0);

    // Setup message filters and masks.
    C1CTRL1bits.WIN = 1; // Allow configuration of masks and filters

    // Set Mask 0 to allow everything.
    CAN1SetMask(0, CAN_MASK_SID(0) & CAN_IGNORE_FILTER_TYPE, CAN_MASK_EID(0));

    // Set Filter 0 to use Mask 0.
    CAN1SetMaskSource(CAN_MASK_FILTER0_MASK0, CAN_MASK_FILTER8_NO_MASK);

    // Set Filter 0 to allow everything.
    CAN1SetFilter(0, CAN_FILTER_SID(0) & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

    // Point filter 0 to our reception buffer (Buffer 1).
    CAN1SetBUFPNT1(CAN_FILTER0_RX_BUFFER1);

    // Enable Filter 0.
    CAN1EnableFilter(0);

    C1CTRL1bits.WIN = 0;

    // Return the modules to specified operating mode.
    CAN1SetOperationMode(CAN_IDLE_CON & CAN_MASTERCLK_FOSC & CAN_CAPTURE_DISABLE & CAN_REQ_OPERMODE_NOR & CAN_SFR_BUFFER_WIN,
            CAN_DO_NOT_CMP_DATABYTES);

    // Enable interrupts for ECAN1
    ConfigIntCAN1(CAN_INVALID_MESSAGE_INT_DIS & CAN_WAKEUP_INT_DIS & CAN_ERR_INT_DIS & CAN_FIFO_INT_DIS & CAN_RXBUF_OVERFLOW_INT_DIS & CAN_RXBUF_INT_EN & CAN_TXBUF_INT_EN,
            CAN_INT_ENABLE & CAN_INT_PRI_7);

    // Configure buffer settings.
    // Specify details on the reception buffer (1) and the transmission buffer (0)
    CAN1SetTXRXMode(0, CAN_BUFFER0_IS_TX & CAN_ABORT_REQUEST_BUFFER0 & CAN_AUTOREMOTE_DISABLE_BUFFER0 & CAN_TX_HIGH_PRI_BUFFER0 &
            CAN_BUFFER1_IS_RX & CAN_ABORT_REQUEST_BUFFER1 & CAN_AUTOREMOTE_DISABLE_BUFFER1 & CAN_TX_HIGH_PRI_BUFFER1);

    /// Set up necessary DMA channels for transmission and reception
    // ECAN1 transmission over DMA2
    OpenDMA2(DMA2_MODULE_ON & DMA2_SIZE_WORD & DMA2_TO_PERIPHERAL & DMA2_INTERRUPT_BLOCK & DMA2_NORMAL & DMA2_PERIPHERAL_INDIRECT & DMA2_CONTINUOUS,
            DMA2_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
            __builtin_dmaoffset(ecan1MsgBuf),
#elif __dsPIC33EP256MC502__
            (unsigned long int)ecan1MsgBuf, // Warning here (cast from pointer to integer of different size) expected, unknown how to fix
#endif
            0ul,
            (uint16_t) & C1TXD,
            7);
    DMA2REQbits.IRQSEL = 0x46; // Attach this DMA to the ECAN1 TX data sent event

    // ECAN1 reception over DMA0
    OpenDMA0(DMA0_MODULE_ON & DMA0_SIZE_WORD & PERIPHERAL_TO_DMA0 & DMA0_INTERRUPT_BLOCK & DMA0_NORMAL & DMA0_PERIPHERAL_INDIRECT & DMA0_CONTINUOUS,
            DMA0_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
            __builtin_dmaoffset(ecan1MsgBuf),
#elif __dsPIC33EP256MC502__
            (unsigned long int)ecan1MsgBuf, // Warning here (cast from pointer to integer of different size) expected, unknown how to fix
#endif
            0ul,
            (uint16_t) & C1RXD,
            7);
    DMA0REQbits.IRQSEL = 0x22; // Attach this DMA to the ECAN1 RX data ready event
}

int Ecan1Receive(CanMessage *msg, uint8_t *messagesLeft)
{
    int foundOne = CB_ReadMany(&ecan1RxCBuffer, msg, sizeof (CanMessage));

    if (messagesLeft) {
        if (foundOne) {
            *messagesLeft = --receivedMessagesPending;
        } else {
            *messagesLeft = 0;
        }
    }

    return foundOne;
}

/**
 * This function transmits a CAN message on the ECAN1 CAN bus.
 * This function is for internal use only as it bypasses the circular buffer. This means that it
 * can squash existing transfers in progress.
 */
void _ecan1TransmitHelper(const CanMessage *message)
{
    uint16_t word0 = 0, word1 = 0, word2 = 0;
    uint16_t sid10_0 = 0, eid5_0 = 0, eid17_6 = 0;
    volatile uint16_t *ecan_msg_buf_ptr = ecan1MsgBuf[message->buffer];

    // Variables for setting correct TXREQ bit
    uint16_t bit_to_set;
    uint16_t offset;
    uint16_t *bufferCtrlRegAddr;

    // Divide the identifier into bit-chunks for storage
    // into the registers.
    if (message->frame_type == CAN_FRAME_EXT) {
        eid5_0 = (message->id & 0x3F);
        eid17_6 = (message->id >> 6) & 0xFFF;
        sid10_0 = (message->id >> 18) & 0x7FF;
        word0 = 1;
        word1 = eid17_6;
    } else {
        sid10_0 = (message->id & 0x7FF);
    }

    word0 |= (sid10_0 << 2);
    word2 |= (eid5_0 << 10);

    // Set remote transmit bits
    if (message->message_type == CAN_MSG_RTR) {
        word0 |= 0x2;
        word2 |= 0x0200;
    }

    ecan_msg_buf_ptr[0] = word0;
    ecan_msg_buf_ptr[1] = word1;
    ecan_msg_buf_ptr[2] = ((word2 & 0xFFF0) + message->validBytes);
    ecan_msg_buf_ptr[3] = ((uint16_t)message->payload[1] << 8 | ((uint16_t)message->payload[0]));
    ecan_msg_buf_ptr[4] = ((uint16_t)message->payload[3] << 8 | ((uint16_t)message->payload[2]));
    ecan_msg_buf_ptr[5] = ((uint16_t)message->payload[5] << 8 | ((uint16_t)message->payload[4]));
    ecan_msg_buf_ptr[6] = ((uint16_t)message->payload[7] << 8 | ((uint16_t)message->payload[6]));

    // Set the correct transfer intialization bit (TXREQ) based on message buffer.
    offset = message->buffer >> 1;
    bufferCtrlRegAddr = (uint16_t *)(&C1TR01CON + offset);
    bit_to_set = 1 << (3 | ((message->buffer & 1) << 3));
    *bufferCtrlRegAddr |= bit_to_set;

    // Keep track of whether we're in a transmission train or not.
    currentlyTransmitting = 1;
}

/**
 * Transmits a CanMessage using the transmission circular buffer.
 */
void Ecan1Transmit(const CanMessage *msg)
{
    // Append the message to the queue.
    // Message are only removed upon successful transmission.
    // They will be overwritten by newer message overflowing
    // the circular buffer however.
    CB_WriteMany(&ecan1TxCBuffer, msg, sizeof (CanMessage), true);

    // If this is the only message in the queue, attempt to
    // transmit it.
    if (!currentlyTransmitting) {
        _ecan1TransmitHelper(msg);
    }
}

void Ecan1GetErrorStatus(uint8_t errors[2])
{
    // Set transmission errors in first array element.
    if (C1INTFbits.TXBO) {
        errors[0] = 3;
    } else if (C1INTFbits.TXBP) {
        errors[0] = 2;
    } else if (C1INTFbits.TXWAR) {
        errors[0] = 1;
    } else {
        errors[0] = 0;
    }

    // Set reception errors in second array element.
    if (C1INTFbits.RXBP) {
        errors[1] = 2;
    } else if (C1INTFbits.RXWAR) {
        errors[1] = 1;
    } else {
        errors[1] = 0;
    }
}

/**
 * This is an interrupt handler for the ECAN1 peripheral.
 * It clears interrupt bits and pushes received message into
 * the circular buffer.
 */
void _ISR _C1Interrupt(void)
{
    // Give us a CAN message struct to populate and use
    CanMessage message;
    uint8_t ide = 0;
    uint8_t srr = 0;
    uint32_t id = 0;
    volatile uint16_t *ecan_msg_buf_ptr;

    // If the interrupt was set because of a transmit, check to
    // see if more messages are in the circular buffer and start
    // transmitting them.
    if (C1INTFbits.TBIF) {

        // After a successfully sent message, there should be at least
        // one message in the queue, so pop it off.
        CB_ReadMany(&ecan1TxCBuffer, &message, sizeof (CanMessage));

        // Check for a buffer overflow. Then clear the entire buffer if there was.
        if (ecan1TxCBuffer.overflowCount) {
            CB_Init(&ecan1TxCBuffer, txDataArray, ECAN1_BUFFERSIZE);
        }

        // Now if there's still a message left in the buffer,
        // try to transmit it.
        if (ecan1TxCBuffer.dataSize >= sizeof (CanMessage)) {
            CanMessage msg;
            CB_PeekMany(&ecan1TxCBuffer, &msg, sizeof (CanMessage));
            _ecan1TransmitHelper(&msg);
        } else {
            currentlyTransmitting = 0;
        }

        C1INTFbits.TBIF = 0;
    }

    // If the interrupt was fired because of a received message
    // package it all up and store in the circular buffer.
    if (C1INTFbits.RBIF) {

        // Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
        if (C1VECbits.ICODE < 32) {
            message.buffer = C1VECbits.ICODE;
        }

        ecan_msg_buf_ptr = ecan1MsgBuf[message.buffer];

        // Clear the buffer full status bit so more messages can be received.
        if (C1RXFUL1 & (1 << message.buffer)) {
            C1RXFUL1 &= ~(1 << message.buffer);
        }

        //  Move the message from the DMA buffer to a data structure and then push it into our circular buffer.

        // Read the first word to see the message type
        ide = ecan_msg_buf_ptr[0] & 0x0001;
        srr = ecan_msg_buf_ptr[0] & 0x0002;

        /* Format the message properly according to whether it
         * uses an extended identifier or not.
         */
        if (ide == 0) {
            message.frame_type = CAN_FRAME_STD;

            message.id = (uint32_t)((ecan_msg_buf_ptr[0] & 0x1FFC) >> 2);
        } else {
            message.frame_type = CAN_FRAME_EXT;

            id = ecan_msg_buf_ptr[0] & 0x1FFC;
            message.id = id << 16;
            id = ecan_msg_buf_ptr[1] & 0x0FFF;
            message.id |= id << 6;
            id = ecan_msg_buf_ptr[2] & 0xFC00;
            message.id |= id >> 10;
        }

        /* If message is a remote transmit request, mark it as such.
         * Otherwise it will be a regular transmission so fill its
         * payload with the relevant data.
         */
        if (srr == 1) {
            message.message_type = CAN_MSG_RTR;
        } else {
            message.message_type = CAN_MSG_DATA;

            message.validBytes = (uint8_t)(ecan_msg_buf_ptr[2] & 0x000F);
            message.payload[0] = (uint8_t)ecan_msg_buf_ptr[3];
            message.payload[1] = (uint8_t)((ecan_msg_buf_ptr[3] & 0xFF00) >> 8);
            message.payload[2] = (uint8_t)ecan_msg_buf_ptr[4];
            message.payload[3] = (uint8_t)((ecan_msg_buf_ptr[4] & 0xFF00) >> 8);
            message.payload[4] = (uint8_t)ecan_msg_buf_ptr[5];
            message.payload[5] = (uint8_t)((ecan_msg_buf_ptr[5] & 0xFF00) >> 8);
            message.payload[6] = (uint8_t)ecan_msg_buf_ptr[6];
            message.payload[7] = (uint8_t)((ecan_msg_buf_ptr[6] & 0xFF00) >> 8);
        }

        // Store the message in the buffer
        CB_WriteMany(&ecan1RxCBuffer, &message, sizeof (CanMessage), true);

        // Increase the number of messages stored in the buffer
        ++receivedMessagesPending;

        // Be sure to clear the interrupt flag.
        C1INTFbits.RBIF = 0;
    }

    // Clear the general ECAN1 interrupt flag.
    IFS2bits.C1IF = 0;

}
