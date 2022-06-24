#ifndef __CLOCKRT__H__
#define __CLOCKRT__H__

#include "global_defines.h"

#ifdef CLOCK_RUNTIME
    #include <iostream>
    #include <string>
    #include <sys/time.h>

/**************************************************************************************
 * Function prototypes for runtime functions, the functions are described in clockRt.cu
 * ************************************************************************************/
    void print_time(timeval *start, std::string text);
#endif

#endif
