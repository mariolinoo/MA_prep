#include "objDet.h"

/****************************************************************************************************************************************************
 * This function runs the detection of objects with DetectNet.
 * Detectnet expects the input image as a float4 array in RGBAf format.
 * The OpenCV Mat image is in uint8 format. It can be easily converted to uchar3.
 * uchar3 and float4 are special types for DetectNet.
 * The image is copied to the GPU memory and converted to uchar3 during this action.
 * It can then be converted to float4 with the command cudaRGB8ToRGBA32().
 * Once the conversion is done, the detection process is started and the detected objects are returned.
 * If the application is from the Steereocam, the input image needs to be converted from GRAY to BGR first.
 * 
 * Input parameters:
 *  - int *numDetections: A pointer to a integer variable in the calling function.
 *  - detectNet *net: A pointer to the DetectNet object of the calling function.
 *  - cv::Mat *image: This is the input image in OpenCV format.
 *  - const uint32_t overlayFlags: These are the flags that define what DetectNet should return 
 *          (i.e. bounding boxes, labels, confidence)
 *  - uchar3* imgBufferRGB: The buffer for the input image for the final conversion. The data of the input image is copied to this with cudaMemcpy2D.
 *  - float4* imgBufferRGBAf: The buffer for the input image that is used by detectnet. imgBufferRGB is converted to float4 with cudaRGB8ToRGBA32.
 * 
 * Return values:
 *  - int *numDetections: After the detection process is finished, it holds the number of detected objects.
 *  - detectNet::Detection *detections: This is a structure that holds the detected objects 
 *          including the elements defined in overlayFlags * 
 * **************************************************************************************************************************************************/
detectNet::Detection* objDet_run(int *numDetections, detectNet* net, cv::Mat *image, const uint32_t overlayFlags, uchar3* imgBufferRGB, float4* imgBufferRGBAf)
{
#ifdef STEEREOCAM
    cv::cvtColor(*image, *image, cv::COLOR_GRAY2BGR);
#endif
    
    detectNet::Detection* detections = NULL;
    
/********************************************************************************
 * Here the OpenCV Mat is copied to the buffer in the GPU memory and converted to float4 format
 * ******************************************************************************/
    cudaMemcpy2D((void*)imgBufferRGB, image->cols*sizeof(uchar3), (void*)image->data, image->step, image->cols*sizeof(uchar3), image->rows, cudaMemcpyHostToDevice);
    cudaRGB8ToRGBA32(imgBufferRGB, imgBufferRGBAf, image->cols, image->rows);
    
/***************************************
 * Here the detection of objects is done
 * *************************************/
    *numDetections = net->Detect((float*)imgBufferRGBAf, (uint32_t)image->cols, (uint32_t)image->rows, &detections, overlayFlags);
    
/********************************
 * Returning the detected objects
 * ******************************/
    return detections;
}

/*******************************************************************************************************************************************
 * This function calculates the 3D coordinates of the detected objects.
 * In case the coordinates should be written to a shared memory, they are copied to an object of type XYZ which is a pointer to the shared memory.
 * First, the center of the detected objects is calculated.
 * 
 * Realsense-application:
 * Once this is done, the function gets the depth value for this specific pixel in the realsense-application.
 * If post-processing filters are activated for the depth map, the pixel needs to be adjusted according to the defined decimation magnitude.
 * If the depth value is greater than 0, the 3D coordinates can be calculated with the help  of the function rs2_deproject_pixel_to_point().
 * rs2_deproject_pixel_to_point() calculates the 3D coordinates with the camera intrinsics of the color camera. 
 * Therefore it needs the pixel of the color image as input.
 * This function returns the X, Y and Z coordinate in a float array.
 * 
 * Steereocam-application:
 * After calculating the center of the detected objects, the coordinates can directly be obtained from cv::Mat xyz.
 * This cv::Mat holds all 3D values of the image and needs to be calculated before this function using cv::reprojectImageTo3D.
 * 
 * The next step is to sort the detected objects by their distance (ascending).
 * At the end of the function, the coordinates can be printed on the color image, logged to console, 
 * or copied to the XYZ object into the shared memory. This depends on the defines #SHOW_IM, #LOGOBJECTS and #USESHM.
 * 
 * Input parameters:
 *  - detectNet::Detection* detections: Structure that holds the detected objects.
 *  - const int numDetections: Number of objects that were detected.
 *  - detectNet* net: A pointer to the DetectNet object of the calling function. This is needed for getting the object class (i.e. TV).
 *  - cv::Mat *image: This is the color image where the objects were detected.
 *  - XYZ *coordinates: A pointer to an object of type XYZ that is used as shared memory.
 *  - sem_t* semptr: A pointer to the semaphore that is used for synchronization of the memory access of the shared memory.
 * 
 * Realsense-specific input parameters:
 *  - rs2::depth_frame *depth: A pointer to the depth frame.
 *  - const rs2_intrinsics color_intrin: This are the intrinsic parameters of the color camera.
 * Steereocam-specific input parameters:
 *  - cv::Mat *xyz: This cv::Mat holds the 3D values of every pixel. This is the output of reprojectImageTo3D.
 * 
 * Return values:
 *  - cv::Mat *image: The color image with bounding boxes, labels and coordinates of the detected objects.
 *  - XYZ *coordinates: The function copies the detected objects and coordinates to this structure that is used as shared memory. 
 * *****************************************************************************************************************************************/
#ifdef USESHM

#ifdef STEEREOCAM
void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, cv::Mat *xyz, cv::Mat *image, XYZ *coordinates, sem_t* semptr)
#else
void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image, XYZ *coordinates, sem_t* semptr)
#endif

#else

#ifdef STEEREOCAM
void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, cv::Mat *xyz, cv::Mat *image)
#else
void objDet_calc_coordinates(detectNet::Detection* detections, const int numDetections, detectNet* net, rs2::depth_frame *depth, const rs2_intrinsics color_intrin, cv::Mat *image)
#endif

#endif
{
    float pixel_point[2], depth_point[3];
    float calc_dist;
#if defined(SHOW_IM) || defined(USESHM)
    int numIters = 0;
#endif
    cv::Scalar color_bb = cv::Scalar(255, 10, 255); // magenta
    cv::Scalar color_text = cv::Scalar(0, 200, 200); // yellow
    tempObjects tempCoords[numDetections];

#ifdef REALSENSE
    float depth_val;
    #ifdef FILTER
        float pixel_pointFilt[2];
    #endif
#endif
        
#ifdef STEEREOCAM
    cv::Point3f depth_steereo;
#endif
    
/**************************************************************************************************************************
 * This loop loops through the detected objects and calculates the depth and 3D coordinates of the center of the objects.
 * ************************************************************************************************************************/
    for(int n=0; n < numDetections; n++)
    {
/************************************************************************************
 * Here the detected objects are copied to a temporary structure of type tempObjects.
 * This structure is used for sorting the objects by their distance.
 * **********************************************************************************/
        tempCoords[n].detection = &detections[n];
        
/**********************************************************************************************************
 * Here the pixel of the center of the detected objects is calculated.
 * In case #COPTER is defined, it means that the image was rotated by 180°.
 * As only the color image was rotated by the main function, the pixel needs to be adjusted for the depth map.
 * ********************************************************************************************************/
#ifdef COPTER                
        pixel_point[0] = WIDTH - detections[n].Left - detections[n].Width()*0.5;
        pixel_point[1] = HEIGHT - detections[n].Top - detections[n].Height()*0.5;
#else
        pixel_point[0] = detections[n].Left + detections[n].Width()*0.5;
        pixel_point[1] = detections[n].Top + detections[n].Height()*0.5;
#endif   

        
#ifdef REALSENSE
/****************************************************************************************************************
 * If the post-processing filters of the depth map are active, the depth image is reduced by the factor DEC_MAGN.
 * Therefore the pixel needs to be adjusted for getting the correct depth value.
 * If they are not active, the above calculated pixel can be used.
 * Also, the distance of the pixel in meters is obtained. This is needed for calculating the 3D coordinates later.
 * **************************************************************************************************************/
    #if defined(FILTER)              
            pixel_pointFilt[0] = pixel_point[0]/DEC_MAGN;
            pixel_pointFilt[1] = pixel_point[1]/DEC_MAGN;         
            depth_val = depth->get_distance((int)pixel_pointFilt[0], (int)pixel_pointFilt[1]);
    #else
            depth_val = depth->get_distance((int)pixel_point[0], (int)pixel_point[1]);
    #endif
#else
/**********************************************************************************
 * For the Steereocam the 3D coordinates can be directly obtained from cv::Mat xyz.
 * ********************************************************************************/
            depth_steereo = xyz->at<cv::Point3f>(pixel_point[1], pixel_point[0]);
            depth_point[0] = depth_steereo.x;
            depth_point[1] = depth_steereo.y;
            depth_point[2] = depth_steereo.z;
#endif

#ifdef REALSENSE
/****************************************************************************************************************
 * If the depth value is greater than 0, the 3D coordinates of the corresponding pixel are calculated.
 * The real distance is then calculated with the formula Dist = SQRT(X*X + Y*Y + Z*Z).
 * As the realsense uses an other coordinate system where the positive Y-axis points down 
 * (see https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20),
 * the value of the Y-coordinate is multiplied with -1 to convert it to the coordinate system used by the copter.
 * In case the #COPTER is defined, the image is rotated by 180° what means that the Y-coordinate is fine, 
 * but the X-coordinate needs to be multiplied with -1.
 * 
 * If the depth value is 0 or lower, it means that the depth value could not be obtained by the camera.
 * In this case, default values (defined in "global_defines.h") are assigned to the coordinates.
 * **************************************************************************************************************/
        if(depth_val > 0)  
        {
/*************************************************************************************************
 * In rs2_deproject_pixel_to_point the 3D coordinates of a pixel in the color image are calculated 
 * by using the camera intrinsics of the color camera and the obtained depth value.
 * ***********************************************************************************************/
            rs2_deproject_pixel_to_point(depth_point, &color_intrin, pixel_point, depth_val);
#endif
 
/**************************************************************************************************************************
 * Here the real distance to the object is calculated with the 3D coordinates and the formula Dist = SQRT(X*X + Y*Y + Z*Z).
 * ************************************************************************************************************************/
            calc_dist = sqrt(depth_point[0] * depth_point[0] + depth_point[1] * depth_point[1] + depth_point[2] * depth_point[2]);
    
/*****************************************************************
 * Here the X or Y axis are multiplied with -1 as described above.
 * ***************************************************************/
#ifdef COPTER
            depth_point[0] = depth_point[0]*(-1);
#else
            depth_point[1] = depth_point[1]*(-1);
#endif
            
#ifdef REALSENSE
        }
#endif

/*************************************************************************************************
 * If the depth value could not be calculated, default values will be assigned to the coordinates.
 * In the realsense calculations, the depth value is 0 in case it could not be calculated.
 * In the steerecam calculations it is infinite.
 * ***********************************************************************************************/
#ifdef STEEREOCAM
        if(isinf(calc_dist)) 
#else
        else
#endif
        {
/**********************************************************
 * Here the default values are assigned as described above.
 * ********************************************************/
            depth_point[0] = DEFAULT_X_COORD;
            depth_point[1] = DEFAULT_Y_COORD;
            depth_point[2] = DEFAULT_Z_COORD;
            calc_dist = DEFAULT_DISTANCE;
        }
        
/****************************************************************************************************
 * Here the 3D coordinates are copied to the temporary structure for sorting the objects by distance.
 * **************************************************************************************************/
        tempCoords[n].depth_point[0] = depth_point[0];
        tempCoords[n].depth_point[1] = depth_point[1];
        tempCoords[n].depth_point[2] = depth_point[2];
        tempCoords[n].calc_dist = calc_dist;
    }

/*********************************************************************
 * In this part, the objects are sorted by their distance (ascending).
 * *******************************************************************/
	for(int i=0; i < numDetections-1; i++)
	{
		for(int j=0; j < numDetections-i-1; j++)
		{
			if(tempCoords[j].calc_dist > tempCoords[j+1].calc_dist)
			{
				const tempObjects det = tempCoords[j];
				tempCoords[j] = tempCoords[j+1];
				tempCoords[j+1] = det;
			}
		}
	}
    
#ifdef LOGOBJECTS
    std::cout << "Found objects:" << std::endl;
#endif    
    
/**********************************************************************************************************
 * If #SHOW_IM is defined, the image is manipulated and the bounding boxes and coordinates are drawn to it.
 * If #USESHM is defined, the coordinates are copied to the XYZ object of the calling function that is used as shared memory..
 * ********************************************************************************************************/
#if defined(SHOW_IM) || defined(USESHM)
/*********************************************************************
 * With #MAXOBJECTS the maximum number of show objects can be limited.
 * *******************************************************************/
    if(numDetections > MAXOBJECTS)
    {
        numIters = MAXOBJECTS;
    }
    else
    {
        numIters = numDetections;
    }
#endif

#ifdef USESHM
	/***************************************************************************************
	 * If the number of found objects is smaller than #MAXOBJECTS, 
	 * the other elements of the shared memory structure get default values.
	 * The semaphore is locked until all objects are written to the shared memory structure.
	 * *************************************************************************************/
	if(sem_wait(semptr) == 0)
    {
	/**********************************************************************************************
	 * In this part, the 3D coordinates and the distance are copied to the shared memory structure.
	 * ********************************************************************************************/
		for(int n=0; n < numIters; n++)
		{
            std::string tempStr = net->GetClassDesc(tempCoords[n].detection->ClassID);
            strcpy(coordinates[n].objType, tempStr.c_str());
            coordinates[n].x_coord = tempCoords[n].depth_point[0];
            coordinates[n].y_coord = tempCoords[n].depth_point[1];
            coordinates[n].z_coord = tempCoords[n].depth_point[2];
            coordinates[n].distance = tempCoords[n].calc_dist;
		}
        if(numIters < MAXOBJECTS)
        {
            for(int n=numIters; n < MAXOBJECTS; n++)
            {
                std::string tempStr = "None";
                strcpy(coordinates[n].objType, tempStr.c_str());
                coordinates[n].x_coord = DEFAULT_X_COORD;
                coordinates[n].y_coord = DEFAULT_Y_COORD;
                coordinates[n].z_coord = DEFAULT_Z_COORD;
                coordinates[n].distance = DEFAULT_DISTANCE;
            }
        }
		/**************************
		 * Releasing the semaphore.
		 * ************************/
		sem_post(semptr);
	}
#endif
#ifdef SHOW_IM 
        for(int n=0; n < numIters; n++)
        {
/*************************************************************************************
 * Here the bounding boxes are drawn and the type of object is printed on the image.
 * ***********************************************************************************/
            cv::Point p1(tempCoords[n].detection->Left + tempCoords[n].detection->Width(), tempCoords[n].detection->Top + tempCoords[n].detection->Height());
            cv::Point p2(tempCoords[n].detection->Left, tempCoords[n].detection->Top);
            cv::putText(*image, net->GetClassDesc(tempCoords[n].detection->ClassID), cv::Point(tempCoords[n].detection->Left, (tempCoords[n].detection->Top + 25)), cv::FONT_HERSHEY_SIMPLEX, 0.75, color_text, 2);
            cv::rectangle(*image, p1, p2, color_bb, 2, 8, 0);

/*****************************************************************************
 * Here the distance and the 3D coordinates are printed on the image.
 * Also, the center of the object is marked with a rectangle of 10x10 pixels.
 * ***************************************************************************/
            cv::Point p3(tempCoords[n].detection->Left + (tempCoords[n].detection->Width() / 2) + 10, tempCoords[n].detection->Top + (tempCoords[n].detection->Height() / 2) + 10);
            cv::Point p4(tempCoords[n].detection->Left + (tempCoords[n].detection->Width() / 2) - 10, tempCoords[n].detection->Top + (tempCoords[n].detection->Height() / 2) - 10);
            cv::rectangle(*image, p3, p4, color_bb, 2, 8, 0);
            cv::putText(*image, "Distance: " + std::to_string(tempCoords[n].calc_dist) + "m", cv::Point(tempCoords[n].detection->Left, (tempCoords[n].detection->Top + 50)), cv::FONT_HERSHEY_SIMPLEX, 0.75, color_text, 2);
            cv::putText(*image, "X: " + std::to_string(tempCoords[n].depth_point[0]) + "m", cv::Point(tempCoords[n].detection->Left, (tempCoords[n].detection->Top + 75)), cv::FONT_HERSHEY_SIMPLEX, 0.75, color_text, 2);
            cv::putText(*image, "Y: " + std::to_string(tempCoords[n].depth_point[1]) + "m", cv::Point(tempCoords[n].detection->Left, (tempCoords[n].detection->Top + 100)), cv::FONT_HERSHEY_SIMPLEX, 0.75, color_text, 2);
            cv::putText(*image, "Z: " + std::to_string(tempCoords[n].depth_point[2]) + "m", cv::Point(tempCoords[n].detection->Left, (tempCoords[n].detection->Top + 125)), cv::FONT_HERSHEY_SIMPLEX, 0.75, color_text, 2);        
        }
#endif 

/********************************************************************************************
 * If #LOGOBJECTS is defined, the objects and their 3D coordinates are logged to the console.
 * ******************************************************************************************/
#ifdef LOGOBJECTS
    for(int n=0; n < numDetections; n++)
    {
        std::cout << "Object " << n << ": " << net->GetClassDesc(tempCoords[n].detection->ClassID) << ", X=" << \
            tempCoords[n].depth_point[0] << "[m] Y=" << tempCoords[n].depth_point[1] << "[m] Z=" << tempCoords[n].depth_point[2] << \
            "[m] Distance=" << tempCoords[n].calc_dist << "[m]" << std::endl;
    }    
#endif
}
