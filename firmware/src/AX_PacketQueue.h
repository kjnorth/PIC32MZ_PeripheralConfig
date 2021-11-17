/* 
 * File:   AX_PacketQueue.h
 * Author: Kodiak North
 * 
 * Description: Defines a queue structure for storing packets that an
 * AX5043/5243 will transmit.
 *
 * Created on November 16, 2021, 3:37 PM
 */

#ifndef AX_PACKETQUEUE_H
#define	AX_PACKETQUEUE_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
#define AX_MAX_ENQUEUED_PACKETS (5u)
#define AX_PACKET_MAX_SIZE (200u)
// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****

    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****

    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    void AX_PacketQueue_Init(void);
    bool AX_EnqueuePacket(uint8_t* txPacket, uint8_t length);
    bool AX_DequeuePacket(uint8_t** packet, uint8_t* size);
    bool AX_IsQueueEmpty(void);
    bool AX_IsQueueFull(void);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* AX_PACKETQUEUE_H */

