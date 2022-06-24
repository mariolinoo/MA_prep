#ifndef __UTILS__H__
#define __UTILS__H__

#include "global_defines.h"
#include <iostream>
#include "detectNet.h"

#ifdef STEEREOCAM
#include "TaraXLDepth.h"
#else
#include <librealsense2/rs.hpp>
#endif

#ifdef USESHM
    #include <sys/ipc.h>
    #include <sys/shm.h>
    #include <fcntl.h>
    #include <semaphore.h>
#endif

// global variable for the handling of the endless loop
extern int flag;

/*******************************************************************************************
 * Function prototypes for the main application, the functions are described in utils.cu
 * *****************************************************************************************/
void sig_handler(int sig);

#ifdef REALSENSE
    #if defined(USESHM) && defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid, int gpio);
    #elif defined(USESHM) && !defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid);
    #elif defined(GPIO_RUNTIME) && !defined(USESHM)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, int gpio);
    #else
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p);
    #endif
#else
    #if defined(USESHM) && defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, sem_t *semptr, XYZ *coordinates, int shmid, int gpio);
    #elif defined(USESHM) && !defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, sem_t *semptr, XYZ *coordinates, int shmid);
    #elif defined(GPIO_RUNTIME) && !defined(USESHM)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, int gpio);
    #else
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList);
    #endif
#endif

/**************************************************************************
 * Function prototypes for the gpio, the functions are described in gpio.cu
 * ************************************************************************/
#ifdef GPIO_RUNTIME
    extern int gpio_release();
#endif

#endif
