#ifndef __GPIO__H__
#define __GPIO__H__

#include "global_defines.h"

#ifdef GPIO_RUNTIME
    #include <errno.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <iostream>

    #define SYSFS_GPIO_DIR "/sys/class/gpio"
    #define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
    #define MAX_BUF 64

    
/********************************************************************************
 * Function prototypes for gpio functions, the functions are described in gpio.cu
 * ******************************************************************************/
    int gpio_export(unsigned int gpio);
    int gpio_unexport(unsigned int gpio);
    int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
    int gpio_set_value(unsigned int gpio, unsigned int value);
    int gpio_init();
    int gpio_release();
#endif
    
#endif
