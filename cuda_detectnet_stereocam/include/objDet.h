#ifndef __OBJDET__H__
#define __OBJDET__H__

#include "global_defines.h"
#include "cudaRGB.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include "detectNet.h"

#ifdef USESHM
#include <semaphore.h>
#endif

#ifdef REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#endif

/*********************************************************************************************
 * This structur defines the type for an object that is used for sorting the detected objects.
 * *******************************************************************************************/
typedef struct tempObjects
{
    detectNet::Detection* detection;
    float depth_point[3];
    float calc_dist;
} tempObjects;


/**********************************************************************************************
 * Function prototypes for object detection functions, the functions are described in objDet.cu
 * ********************************************************************************************/
detectNet::Detection* objDet_run(int *numDetections, detectNet* net, cv::Mat *image, const uint32_t overlayFlags, uchar3* imgBufferRGB, float4* imgBufferRGBAf);

#ifdef USESHM
    #ifdef STEEREOCAM
        void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, cv::Mat *xyz, cv::Mat *image, XYZ *coordinates, sem_t* semptr);
    #else
        void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image, XYZ *coordinates, sem_t* semptr);
    #endif
#else
    #ifdef STEEREOCAM
        void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, cv::Mat *xyz, cv::Mat *image);
    #else
        void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image);
    #endif
#endif

#endif
