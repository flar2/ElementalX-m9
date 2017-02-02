/* 
 * File:   timeQueue.h
 * Author: W0017688
 *
 * Created on February 22, 2016, 12:32 AM
 */

#ifndef TIMEQUEUE_H
#define	TIMEQUEUE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "platform.h"

#define NUM_TIMERS 11
    
    typedef enum
    {
        TIMER_CC_DEBOUNCE = 0,
        TIMER_PD_DEBOUNCE,
        TIMER_STATE,
    } Timer;
    
    FSC_U32 timeQueue[NUM_TIMERS][2];
    
    // TODO: Make sure elapsed time is available in Linux kernel (surprised if not)
    
    // Initialise
    void timeQueueInit(void);
    
    // Queues timer with specified time
    void timerQueue(Timer timer, FSC_U32 time);
    
    // Returns true if timer has expired
    BOOL timerCheck(Timer timer);
    
    // Called after interrupt. Subtracts lowest timer from all timers
    void expireTimer(void);
    
    // Same as Queue Timer for 0
    void removeTimer(Timer timer);
    
    
    


#ifdef	__cplusplus
}
#endif

#endif	/* TIMEQUEUE_H */

