/* 
 * File:   Utils.h
 * Author: Kodiak North
 * 
 * Description: File to hold all utility macros that a developer could possibly
 * need.
 *
 * Created on July 21, 2021, 1:48 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ARRAY_SIZE(x)           (sizeof(x) / sizeof(x[0]))
#define ABS(x)                  ((x < 0) ? -x : x)
#define IS_POW_OF_TWO(x)        ((x != 0) && (((x & (x - 1)) == 0)))
#define GET_LSB(x)              ((x > 0) ? (x & (~x + 1)) : 0)
#define SHIFT_BIT(x)            (1 << x) // shifts an int into its corresponding bit position, uint8_t has bits [7:0]
#define CHECK_BIT(var,pos)      ((var) & (1<<(pos)))
#define SET_BIT(var,pos)        ((var) |= (1<<(pos)))
#define CLR_BIT(var,pos)        ((var) &= ~(1<<(pos)))


#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

