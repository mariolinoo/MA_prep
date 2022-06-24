#include "gpio.h"

#ifdef GPIO_RUNTIME

/********************************************************************************************
 * The gpio_export, gpio_unexport, gpio_set_dir and gpio_set_value functions were taken from:
 * https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
 * Some adaptions were made for warnings removal and error handling.
 * 
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************************/

/************************************************************
 * This function exports the given GPIO.
 * 
 * Input parameters:
 *  - unsinged int gpio: GPIO number that should be exported.
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 ************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd, len, ret;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export open");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
    if(len < 0)
    {
        perror("gpio/export snprintf");
        return len;
    }
	ret = write(fd, buf, len);
    if(ret == -1 && errno != EBUSY)
    {
        perror("gpio/export write");
        return ret;
    }
	close(fd);
 
	return 0;
}

/**************************************************************
 * This function unexports the given GPIO.
 * 
 * Input parameters:
 *  - unsinged int gpio: GPIO number that should be unexported.
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 **************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len, ret;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/unexport open");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
    if(len < 0)
    {
        perror("gpio/unexport snprintf");
        return len;
    }
	ret = write(fd, buf, len);
    if(ret == -1)
    {
        perror("gpio/unexport write");
        return ret;
    }
	close(fd);
 
	return 0;
}

/************************************************************
 * This function sets the direction of the given GPIO.
 * 
 * Input parameters:
 *  - unsinged int gpio: GPIO number that should be exported.
 *  - unsigned int out_flag: Flag that sets the direction. 
 *          Value 1 means OUT, 0 means IN.
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 ************************************************************/
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd, len, ret;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
    if(len < 0)
    {
        perror("gpio/direction snprintf");
        return len;
    }
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction open");
		return fd;
	}
 
	if (out_flag)
    {
		ret = write(fd, "out", 4);
        if(ret == -1)
        {
            perror("gpio/direction write");
            return ret;
        }
    }
	else
    {
		ret = write(fd, "in", 3);
        if(ret == -1)
        {
            perror("gpio/direction write");
            return ret;
        }
    }
 
	close(fd);
	return 0;
}

/****************************************************************************
 * This function sets the value of the given GPIO.
 * 
 * Input parameters:
 *  - unsinged int gpio: GPIO number that should be exported.
 *  - unsigned int value: Value for the GPIO output (1 or 0 for HIGH or LOW).
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 ****************************************************************************/
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len, ret;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
    if(len < 0)
    {
        perror("gpio/set-value snprintf");
        return len;
    }
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value open");
		return fd;
	}
 
	if (value)
    {
		ret = write(fd, "1", 2);
        if(ret == -1)
        {
            perror("gpio/set-value write");
            return ret;
        }
    }
	else
    {
		ret = write(fd, "0", 2);
        if(ret == -1)
        {
            perror("gpio/set-value write");
            return ret;
        }
    }
 
	close(fd);
	return 0;
}

/*********************************************************************
 * This function initializes the defined GPIOs.
 * First, it exports them, which makes them usable by the application. 
 * If they are already exported, this is no critical error itself, 
 * but it could mean that they are used by another application.
 * Next, their direction is set to OUT.
 * And finally, their output value is set to 0.
 * 
 * Input parameters:
 *  - None
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 * *******************************************************************/
int gpio_init()
{
    int ret;
    
/*************************************************************************************************
 * Exporting the GPIOs. In case of an error, the error message is printed in gpio_export function.
 * ***********************************************************************************************/
    ret = gpio_export(PIN_FPS);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_export(PIN_OBJDET);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_export(PIN_FRAME);
    if(ret < 0)
    {
        return ret;
    }
    
#ifdef FILTER
    ret = gpio_export(PIN_FILTER);
    if(ret < 0)
    {
        return ret;
    }
#endif
    
/*********************************************************************************
 * Setting the direction of the GPIOs. 1 means output.
 * In case of an error, the error message is printed in the gpio_set_dir function.
 * *******************************************************************************/
    ret = gpio_set_dir(PIN_FPS, 1);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_set_dir(PIN_OBJDET, 1);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_set_dir(PIN_FRAME, 1);
    if(ret < 0)
    {
        return ret;
    }
    
#ifdef FILTER
    ret = gpio_set_dir(PIN_FILTER, 1);
    if(ret < 0)
    {
        return ret;
    }
#endif

/***********************************************************************************
 * Setting the output value of the GPIOs to 0.
 * In case of an error, the error message is printed in the gpio_set_value function.
 * *********************************************************************************/
    ret = gpio_set_value(PIN_FPS, 0);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_set_value(PIN_OBJDET, 0);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_set_value(PIN_FRAME, 0);
    if(ret < 0)
    {
        return ret;
    }
    
#ifdef FILTER
    ret = gpio_set_value(PIN_FILTER, 0);
    if(ret < 0)
    {
        return ret;
    }
#endif
    
/*********************************
 * Return 0 if there was no error.
 * *******************************/
    return 0;
}

/****************************************************************
 * This function unexports the used GPIOs.
 * This step is necessary to release them for other applications.
 * 
 * Input parameters:
 *  - None
 * 
 * Return values:
 *  - 0: No errors occured.
 *  - -1: An error occured.
 * **************************************************************/
int gpio_release()
{
    int ret;
    
/******************************************************************************
 * Unexporting the defined pins. 
 * In case of an error, the error message is printed in gpio_unexport function.
 * ****************************************************************************/
    ret = gpio_unexport(PIN_FPS);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_unexport(PIN_OBJDET);
    if(ret < 0)
    {
        return ret;
    }
    ret = gpio_unexport(PIN_FRAME);
    if(ret < 0)
    {
        return ret;
    }
    
#ifdef FILTER
    ret = gpio_unexport(PIN_FILTER);
    if(ret < 0)
    {
        return ret;
    }
#endif
    
/*********************************
 * Return 0 if there was no error.
 * *******************************/
    return 0;
}

#endif
