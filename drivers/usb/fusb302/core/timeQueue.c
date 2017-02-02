#include "timeQueue.h"    

//TODO: Make sure getting an interrupt during a function does not break everything
// Maybe disable timer interrupt

// Initialise
    void timeQueueInit(void)
    {
        FSC_U32 i;
        for(i=0; i<NUM_TIMERS; i++)
        {
            timeQueue[i][0] = i;
            timeQueue[i][1] = 0;
        }
    }    

// Queues timer with specified time
    void timerQueue(Timer timer, FSC_U32 time)
    {
        FSC_U32 pos_time = NUM_TIMERS;
        FSC_U32 pos_timer=0;
        FSC_S32 elapsed_time = 0;
        FSC_S32 temp;
        FSC_U32 lowest_time = 0;
        FSC_U32 i;
        
        /* 
         * 0. Sub Elapsed Time
         * 1. Find positions 
         2. Remove timer entry
         3. Shift timers

         5. Add new timer entry
         6. Reset timer interrupt to lowest time*/
        

        for(i=0; i<NUM_TIMERS; i++)
        {
            if(timeQueue[i][1] != 0)
            {
                lowest_time = timeQueue[i][1];
                break;
            }
        }
        
        elapsed_time = lowest_time - platform_get_current_time(); //(should return 0 for disabled timer);  
        if(elapsed_time < 0)
        {
            elapsed_time = 0;
        }
        
        lowest_time = 0;
        
        //TODO: Case for time greater than everything in queue (==NUM_TIMERS)
        // -Add extra space to buffer for shifting?
        for(i=0; i<NUM_TIMERS; i++)
        {
            temp = (timeQueue[i][1] - elapsed_time);    // Subtract elapsed time
            if(temp !=0)
            {
                timeQueue[i][1] = temp;
                if((temp < time) && (lowest_time == 0))
                {
                    lowest_time = temp;
                }
            }
            else
            {
                timeQueue[i][1] = 0;
            }
            
            if(timeQueue[i][0] == timer)
            {
                pos_timer = i;
            }
            if(timeQueue[i][1] <= time)
            {
                pos_time = i;
            }
        }
        
        if((time < lowest_time) && (time != 0))
        {
            lowest_time = time;
        }
        
        if(pos_time<pos_timer)
        {
         //-Shift down to pos_timer
            for(i = pos_timer - 1; i >= pos_time; i--)
            {
                timeQueue[i+1][0] = timeQueue[i][0];
                timeQueue[i+1][1] = timeQueue[i][1];
            }
        }
        else if (pos_time>pos_timer)
        {
         //-Shift up to pos_timer
            for(i = pos_timer + 1; i <= pos_time; i++)
            {
                timeQueue[i-1][0] = timeQueue[i][0];
                timeQueue[i-1][1] = timeQueue[i][1];
            }
        }
        else
        {
         //-already in the correctly place, do nothing
        }
        
        timeQueue[pos_time][0] = timer;
        timeQueue[pos_time][1] = time;
        
        /* Interrupt timer set for last_zero + 1
         * -unless equal to NUM_TIMERS + 1, then it's position 0
         - but if this is NUM_TIMERS, disable the timer*/
        if((lowest_time == 0))
        {
            platform_disable_interrupt_timer();
        }
        else
        {
            platform_set_interrupt_timer(lowest_time);
        }
        
        
    }
    
    // Returns true if timer has expired
    BOOL timerCheck(Timer timer)    // Expects input to be <= NUM_TIMERS
    {
        //TODO: Could be improved
        FSC_U32 i = 0;
        do
        {
        }while(timeQueue[i++][0] != timer);
        
        return !timeQueue[i-1][1];
    }
    
    // Called after interrupt. Subtracts lowest timer from all timers
    void expireTimer(void)
    {
        FSC_S32 temp;
        FSC_U32 lowest_time = 0;
        FSC_U32 i;
        FSC_U32 elapsed_time = 0;
        
        for(i=0; i<NUM_TIMERS; i++)
        {
            if(timeQueue[i][1] != 0)
            {
                lowest_time = timeQueue[i][1];
                break;
            }
        }
        
        elapsed_time = lowest_time - platform_get_current_time();
        lowest_time = 0;
        
        for(i=0; i<NUM_TIMERS; i++)
        {
            temp = (timeQueue[i][1] - elapsed_time);    // Subtract elapsed time
            if(temp !=0)
            {
                timeQueue[i][1] = temp;
                if(lowest_time == 0)
                {
                    lowest_time = temp;
                }
            }
            else
            {
                timeQueue[i][1] = 0;
            }
        }
        
        
                /* Interrupt timer set for lowest_pos + 1 time
         - but if it is NUM_TIMERS, disable the timer*/
        if(lowest_time == 0)
        {
            platform_disable_interrupt_timer();
        }
        else
        {
            platform_set_interrupt_timer(lowest_time);
        }
    }
    
    // Same as Queue Timer for 0
    void removeTimer(Timer timer)
    {
        timerQueue(timer, 0);
    }
