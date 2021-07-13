/* 
 * File:   Print.h
 * Author: Kodiak North
 * 
 * Description: The Print module uses UART5 in non-blocking mode to transmit
 * printf style messages to another unit to display over a serial port. When
 * a message is received, the other unit must respond with PRINT_ACK (defined
 * in Print.c) to inform the sending device that the message was received.
 * 
 * Usage: The Print_EnqueueMsg function adds a message to a queue to be
 * transmitted once all other transmissions are complete. Since this module is
 * non-blocking, the Print_Task function must be called repetitively to handle
 * the sending and receiving of messages and acknowledgments.
 *
 * Created on July 13, 2021, 10:00 AM
 */

#ifndef PRINT_H
#define	PRINT_H

#include <stdarg.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

    void Print_Init(void);
    bool Print_EnqueueMsg(const char* fmt, ...); // call with a message to print
    bool Print_IsQueueFull(void);
    void Print_Task(void); // call continuously to empty print queue as messages are enqueued

#ifdef	__cplusplus
}
#endif

#endif	/* PRINT_H */

