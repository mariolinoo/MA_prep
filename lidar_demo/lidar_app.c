/*
 * @file lidar_app.c
 * @date 14.03.2022
 * @version v0.1
 * @author Roman Beneder (RB), Patrick Schmitt (PS)
 *
 * @brief 
 *
 * Todo as this software is currently only for demo purposes!
 * 		- Implement reasonable error detection and handling
 */

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>

#include <wait.h>
#include <sys/types.h>
#include <termios.h> /* COM libraries for Linux OS */
#include "i2c/i2c.h"

#if defined(SHAREDMEM)
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <semaphore.h>
#define KEY_RAD 2345
#define SEM_NAME_RAD "sem_rad"

typedef struct XYZ_RAD_POS_V2
{
	float coords[15];
} XYZ_RAD_POS_V2;
sem_t* semptr;
key_t key;
XYZ_RAD_POS_V2 *coordinates;
int shmid;
#endif

int exit_flag = 0;
/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
 */


/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
 */

/*
==============================================================================
   4. DATA
==============================================================================
 */


/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
 */


/*
==============================================================================
   6. LOCAL FUNCTIONS
==============================================================================
 */


/*
==============================================================================
   7. MAIN METHOD
==============================================================================
 */

int main(int argc, char** argv)
{

	int i2c_bus;
	ssize_t ret;
	I2CDevice lidar;

	/* Open i2c bus /dev/i2c-0 */
	if ((i2c_bus = i2c_open("/dev/i2c-1")) == -1) {

		/* Error process */
	}

	lidar.bus = i2c_bus;
	lidar.addr = 0x62;
	lidar.tenbit = 0;
	lidar.delay = 1;
	lidar.flags = 0;
	lidar.page_bytes = 8;
	lidar.iaddr_bytes = 1;

	printf("I2C-Bus initialized\n");

	char data = 0x00;

	ret = i2c_ioctl_write(&lidar, 0x00, &data, 1);
	if (ret == -1 || (size_t)ret != 1)
	{
		printf("ERROR_SEND_I2C\n");
	}

	data = 0x80;

	ret = i2c_ioctl_write(&lidar, 0x02, &data, 1);
	if (ret == -1 || (size_t)ret != 1)
	{
		printf("ERROR_SEND_I2C\n");
	}
	data = 0x08;

	ret = i2c_ioctl_write(&lidar, 0x04, &data, 1);
	if (ret == -1 || (size_t)ret != 1)
	{
		printf("ERROR_SEND_I2C\n");
	}

	data = 0x00;

	ret = i2c_ioctl_write(&lidar, 0x1c, &data, 1);
	if (ret == -1 || (size_t)ret != 1)
	{
		printf("ERROR_SEND_I2C\n");
	}

	char distanceArray[2];
	int bias_count = 100;

	while(1)
	{
		if(bias_count < 100){
			data = 0x03;
			bias_count++;
		}
		else{
			data = 0x04;
			bias_count = 0;
		}
		ret = i2c_ioctl_write(&lidar, 0x00, &data, 1);
		if (ret == -1 || (size_t)ret != 1)
		{
			printf("ERROR_SEND_I2C\n");
		}

		// Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
		if ((i2c_read(&lidar, 0x8f, distanceArray, 2)) != 2) {

			printf("ERROR_RECEIVE_I2C\n");
		}
		// Shift high byte and add to low byte
		int distance = (distanceArray[0] << 8) + distanceArray[1];

		if(distance >= 10)
		{
			printf("Got distance data:%d\n",distance);
		}
		usleep(100000);
	}

#if defined(SHAREDMEM) && defined(LINUX) && defined(FLOAT_MODE)
	if(semptr != NULL)
	{
		if(sem_close(semptr) == -1)
		{
			perror("sem_close");
			return EXIT_FAILURE;
		}
		if(sem_unlink("sem_rad") == -1)
		{
			perror("sem_unlink");
			return EXIT_FAILURE;
		}
		if(coordinates != NULL)
		{
			if(shmdt(coordinates) == -1)
			{
				perror("shmdt");
				return EXIT_FAILURE;
			}
		}
		if(shmid != -1)
		{
			if(shmctl(shmid, IPC_RMID, NULL) == -1)
			{
				perror("shmctl");
				return EXIT_FAILURE;
			}
		}
	}
#endif
	return EXIT_SUCCESS;
}
