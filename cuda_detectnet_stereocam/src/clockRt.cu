#include "clockRt.h"

#ifdef CLOCK_RUNTIME

/************************************************************************************************************
 * This function gets the current time and calculates the time difference in seconds to the given start-time.
 * It then prints the time difference with the given text to the console.
 * 
 * Input parameters:
 *  - timeval *start: Timeval object that holds the start time for calculating the difference.
 *  - std::string text: Text that printing the time difference to the console.
 * 
 * Return values:
 *  - None
 * **********************************************************************************************************/

void print_time(timeval *start, std::string text)
{
    timeval end;
    float deltaTime;
    int ret;
    
    ret = gettimeofday(&end, 0);
    if(ret < 0)
    {
        std::cerr << "Could not get current end time..." << std::endl;
    }
    
/****************************************************************************************
 * Calculating the difference in seconds and adding the difference in microseconds to it.
 * According to https://linuxhint.com/gettimeofday_c_language/ 
 * the microseconds are additional to the seconds calculation.
 * **************************************************************************************/    
    deltaTime = (float)(end.tv_sec - start->tv_sec + (end.tv_usec - start->tv_usec) * 1e-6);  
    
    std::cout << text << deltaTime << std::endl;
}
#endif
