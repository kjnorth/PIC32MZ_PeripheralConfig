/* 
 * File:   IMU_HWT901B.h
 * Author: Kodiak North
 * 
 * Description: Implements an API to interface with the WitMotion HWT901B IMU.
 * 
 * For all data arrays, index 0 is x-axis, 1 is y-axis, and 2 is z-axis.
 * For angle data array, index 0 is roll, 1 is pitch, and 2 is yaw.
 *
 * Created on September 10, 2021, 8:42 AM
 */

#ifndef IMU_HWT901B_H
#define	IMU_HWT901B_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****

// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif
    uint16_t numUpdates;
    // **** GLOBAL TYPEDEFS ****
    typedef struct {
        float Pitch;
        float Roll;
        float PitchOffset;
        float RollOffset;
    } X_PL_IMU_t;
    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****
    X_PL_IMU_t RightIMU;
    X_PL_IMU_t LeftIMU;
    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    void IMU_Init(void);
    void IMU_Config(void /*reg to config?*/);
    void IMU_SampleTask(void); // call continuously in main loop
    
    float IMU_RollGet(void);
    float IMU_PitchGet(void);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* IMU_HWT901B_H */

