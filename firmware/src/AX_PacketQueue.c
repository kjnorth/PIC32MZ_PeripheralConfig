/* 
 * File:   AX_PacketQueue.c
 * Author: Kodiak North
 * 
 * Description: Defines a queue structure for storing packets that an
 * AX5043/5243 will transmit.
 *
 * Created on November 16, 2021, 3:39 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include <string.h>

#include "AX_PacketQueue.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****

// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef struct {
    uint8_t packetQueue[AX_MAX_ENQUEUED_PACKETS][AX_FIFO_MAX_SIZE];
    uint8_t packetSize[AX_MAX_ENQUEUED_PACKETS]; // holds the length of each packet at the corresponding index
    uint8_t index; // current index in the packetQueue
    uint8_t size; // number of packets enqueued
} transmit_queue_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
static transmit_queue_t TxQueue;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****

// **** END MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void AX_PacketQueue_Init(void) {
    TxQueue.index = 0;
    TxQueue.size = 0;
}

bool AX_EnqueuePacket(uint8_t* txPacket, uint8_t length) {
    bool returnVal = false;
    if ((length <= AX_FIFO_MAX_SIZE) && !AX_IsQueueFull()) {
        // copy the message into the queue
        uint8_t packetToAddIndex = TxQueue.index + TxQueue.size;
        if (packetToAddIndex >= AX_MAX_ENQUEUED_PACKETS) {
            // perform the modulo operation when necessary
            packetToAddIndex = packetToAddIndex % AX_MAX_ENQUEUED_PACKETS;
        }
        memcpy(TxQueue.packetQueue[packetToAddIndex], txPacket, length);
        TxQueue.packetSize[packetToAddIndex] = length;
        TxQueue.size++;
        returnVal = true;
    }
    return returnVal;
}

bool AX_DequeuePacket(uint8_t** packet, uint8_t* size) {
    bool returnVal = false;
    if (!AX_IsQueueEmpty()) {
        *packet = TxQueue.packetQueue[TxQueue.index];
        *size = TxQueue.packetSize[TxQueue.index];
        TxQueue.index++;
        if (TxQueue.index >= AX_MAX_ENQUEUED_PACKETS) {
            // perform the modulo operation when necessary
            TxQueue.index = TxQueue.index % AX_MAX_ENQUEUED_PACKETS;
        }
        TxQueue.size--;
        returnVal = true;
    }
    return returnVal;
}

bool AX_IsQueueEmpty(void) {
    return (TxQueue.size <= 0);
}

bool AX_IsQueueFull(void) {
    return (TxQueue.size >= AX_MAX_ENQUEUED_PACKETS);
}

// **** MODULE FUNCTION IMPLEMENTATIONS ****

// **** END MODULE FUNCTION IMPLEMENTATIONS ****
