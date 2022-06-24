#ifndef __REALSENSE__H__
#define __REALSENSE__H__

#include "global_defines.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <math.h>
#include <signal.h>
#include <sys/time.h>
#include <librealsense2/rs.hpp>
#include "detectNet.h"

#ifdef USESHM
    #include <sys/ipc.h>
    #include <sys/shm.h>
    #include <fcntl.h>
    #include <semaphore.h>
#endif

/*******************************************************************************************
 * Function prototypes for the main application, the functions are described in utils.cu
 * *****************************************************************************************/
extern void sig_handler(int sig);
#if defined(USESHM) && defined(GPIO_RUNTIME)
extern int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid, int gpio);
#elif defined(USESHM) && !defined(GPIO_RUNTIME)
extern int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, sem_t *semptr, XYZ *coordinates, int shmid);
#elif defined(GPIO_RUNTIME) && !defined(USESHM)
extern int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p, int gpio);
#else
extern int cleanup(uchar3* imgBufferRGB, float4* imgBufferRGBAf, rs2::pipeline *p);
#endif

/**********************************************************************************************
 * Function prototypes for realsense camera functions, the functions are described in camera.cu
 * ********************************************************************************************/
extern void camera_init(rs2::config *cfg, rs2::pipeline *p, rs2::pipeline_profile *prof);
extern rs2_intrinsics get_camera_intrinsics(rs2::frameset *frames, rs2::pipeline *p);
extern rs2::depth_frame get_frames(rs2::frameset *frames, rs2::pipeline *p, rs2::align *align_to_color, cv::Mat *image);

#ifdef FILTER
    extern void filters_init(rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter);
    extern rs2::depth_frame filter_depth_frame(rs2::depth_frame *depth, rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::disparity_transform *depth_to_disparity, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter, rs2::disparity_transform *disparity_to_depth, rs2::align *align_to_color);
#endif
    
#ifdef MAXLASERPWR
    extern void set_max_laser_pwr(rs2::pipeline_profile *prof);
#endif   
    
#ifdef SHOW_DEPTH
    extern void show_depth_frame(rs2::colorizer *color_map, rs2::depth_frame *depth, std::string text);
#endif


/**********************************************************************************************
 * Function prototypes for object detection functions, the functions are described in objDet.cu
 * ********************************************************************************************/
extern detectNet::Detection* objDet_run(int *numDetections, detectNet* net, cv::Mat *image, const uint32_t overlayFlags, uchar3* imgBufferRGB, float4* imgBufferRGBAf);

#ifdef USESHM
    extern void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image, XYZ *coordinates, sem_t* semptr);
#else
    extern void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image);
#endif
        
        
/********************************************************************************
 * Function prototypes for gpio functions, the functions are described in gpio.cu
 * ******************************************************************************/
#ifdef GPIO_RUNTIME
    extern int gpio_export(unsigned int gpio);
    extern int gpio_unexport(unsigned int gpio);
    extern int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
    extern int gpio_set_value(unsigned int gpio, unsigned int value);
    extern int gpio_init();
    extern int gpio_release();
#endif

    
/**************************************************************************************
 * Function prototypes for runtime functions, the functions are described in clockRt.cu
 * ************************************************************************************/
#ifdef CLOCK_RUNTIME
    extern void print_time(timeval *start, std::string text);
#endif

#endif
