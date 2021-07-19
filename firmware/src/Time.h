/* 
 * File:   Time.h
 * Author: Kodiak North
 * 
 * Description: The Time module is used to track time in milliseconds for use
 * throughout the program. It requires that the CORE TIMER module be configured
 * to generate a periodic interrupt every 1 millisecond (i.e. at 1 kHz).
 *
 * Created on July 19, 2021, 11:56 AM
 */

#ifndef TIME_H
#define	TIME_H

#ifdef	__cplusplus
extern "C" {
#endif

    void Time_Init(void);
    uint32_t Time_GetMs(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TIME_H */

