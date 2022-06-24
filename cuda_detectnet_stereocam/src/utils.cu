#include "utils.h"

/******************************************************
 * This is a signal handler that catches SIGINT signals
 * and tells the application to exit.
 * ****************************************************/
void sig_handler(int sig)
{
    flag = 1;
}

#ifdef REALSENSE
/***********************************************************
 * This function cleans up the used and allocated resources.
 * *********************************************************/
    #if defined(USESHM) && defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid, int gpio)
    #elif defined(USESHM) && !defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid)
    #elif defined(GPIO_RUNTIME) && !defined(USESHM)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, int gpio)
    #else
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p)
    #endif
#else
    #if defined(USESHM) && defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, sem_t *semptr, XYZ *coordinates, int shmid, int gpio)
    #elif defined(USESHM) && !defined(GPIO_RUNTIME)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, sem_t *semptr, XYZ *coordinates, int shmid)
    #elif defined(GPIO_RUNTIME) && !defined(USESHM)
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList, int gpio)
    #else
    int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, TaraXLSDK::TaraXLCamList *taraxlCamList)
    #endif
#endif
{
    int retval = EXIT_SUCCESS;
/**************************************************
 * The allocated memory on the GPU is released here
 * ************************************************/
#ifdef LOGCONSOLE
    std::cout << "Releasing allocated GPU memory..." << std::endl;
#endif
    if(imgBufferRGB != NULL)
    {
        if(cudaFree(imgBufferRGB) == cudaErrorInvalidValue)
        {
            std::cerr << "Error freeing imgBufferRGB..." << std::endl;
            retval = EXIT_FAILURE;
        }
    }
    if(imgBufferRGBAf != NULL)
    {
        if(cudaFree(imgBufferRGBAf) == cudaErrorInvalidValue)
        {
            std::cerr << "Error freeing imgBufferRGBAf..." << std::endl;
            retval = EXIT_FAILURE;
        }
    }
       
#ifdef LOGCONSOLE
    std::cout << "Stopping camera..." << std::endl;
#endif
/********************************
 * This command stops the camera.
 * ******************************/
 #ifdef REALSENSE
    try
    {
        p->stop();
    }
    catch(const rs2::error e)
    {
        // camera was not started yet, no action needed in cleanup
    }
#else
    for(int i = 0 ; i < taraxlCamList->size() ; i++)
    {
        taraxlCamList->at(i).disconnect();
    }
#endif

/****************************************************************************
 * The semaphore is closed and unlinked
 * (removed from the kernel once all applications that use it are shut down).
 * Also, the shared memory is detached and removed.
 * **************************************************************************/
#ifdef USESHM
    #ifdef LOGCONSOLE
    std::cout << "Closing semaphore and detaching shared memory..." << std::endl;
    #endif
    if(semptr != NULL)
    {
        if(sem_close(semptr) == -1)
        {
            perror("sem_close");
            retval = EXIT_FAILURE;
        }
    #ifdef REALSENSE
        if(sem_unlink("sem_rs") == -1)
    #else
         if(sem_unlink("sem_sc") == -1)   
    #endif
        {
            perror("sem_unlink");
            retval = EXIT_FAILURE;
        }
    }
    if(coordinates != NULL)
    {
        if(shmdt(coordinates) == -1)
        {
            perror("shmdt");
            retval = EXIT_FAILURE;
        }
    }
    if(shmid != -1)
    {
        if(shmctl(shmid, IPC_RMID, NULL) == -1)
        {
            perror("shmctl");
            retval = EXIT_FAILURE;
        }
    }
#endif
    
#ifdef GPIO_RUNTIME
    #ifdef LOGCONSOLE
    std::cout << "Releasing GPIOs..." << std::endl;
    #endif
    
/***************************************************************************
 * Here the GPIOs are unexported which releases them for other applications.
 * In case of an error, the error message is printed in the subfunctions.
 * *************************************************************************/
    if(gpio == 1)
    {
        if(gpio_release() != 0)
        {
            retval = EXIT_FAILURE;
        }
    }
#endif

    return retval;
}
