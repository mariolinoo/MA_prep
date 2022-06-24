#include "realsense.h"

// global variable for the handling of the endless loop
int flag;

/*******************************************************************************************************************
 * This is the main function of the realsense application.
 * It installs a signal handler to the signal SIGINT, initializes the camera module and creates a detectnet object.
 * If defined, it also initializes the gpio, clock and shared memory, colorizer for the depth map, 
 * filters, and sets the maximum laser power.
 * 
 * In an endless loop it obtains the color and depth frames from the realsense camera.
 * It then calls the necessary functions to detect objects and calculate its coordinates.
 * If defined, it also logs the runtime information using the clock, 
 * controls the gpios for external runtime analysis with an oscilloscope for example, and shows the depth frame.
 * If #USESHM is defined, the objects are written to the shared memory in the object detection functions, 
 * or, if no object was found some default values are written to the shared memory by the main function.
 * 
 * The main function includes the following options - these are described and can be activated in global_defines.h:
 * #SHOW_IM, #SHOW_DEPTH, #LOGCONSOLE, #LOGOBJECTS, #FILTER, #MAXLASERPWR,
 * #COPTER, #GPIO_RUNTIME, #CLOCK_RUNTIME, #USESHM, #SAVEPIC, #MAXOBJECTS
 * 
 * The application exits if a SIGINT was caught or, in case #SAVEPIC is defined, it ends after 1000 loops.
 ******************************************************************************************************************/
int main(int argc, char** argv)
{
    // variable for the return value of the application
    int retval = EXIT_SUCCESS;
    
    // installing signal handler
    if(signal(SIGINT, sig_handler) == SIG_ERR)
    {
        perror("signal");
        return EXIT_FAILURE;
    }
    
/***********************************************************
 * Variables for object detection.
 * The obtained color image is copied to these image buffers 
 * and converted to the correct format needed by detectnet.
 * *********************************************************/
    uchar3* imgBufferRGB = NULL;
    float4* imgBufferRGBAf = NULL;
    cudaMalloc((void**)&imgBufferRGB, WIDTH * sizeof(uchar3) * HEIGHT);
    cudaMalloc((void**)&imgBufferRGBAf, WIDTH * sizeof(float4) * HEIGHT);
    
/**********************************************************
 * Variables used for interfacing with the realsense camera
 * ********************************************************/
    rs2::pipeline p;
    rs2::config cfg;
    rs2::pipeline_profile prof;    
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::frameset frames;
#ifdef FILTER
    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thr_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
#endif

/***************************************************************************************************
 * Variables used for interfacing with detectnet
 * cmdLine holds the attributes from calling the application in the console.
 * With the attribute "--network=" the model for the object-detection can be configured.
 * The possible values depend on which models are downloaded with the Jetson-Inference Model-Loader.
 * This application was tested with the networks "mobilenet-v1" and "mobilenet-v2".
 * If no attribute is given to the application, the default model is mobilenet-v2.
 * In overlayFlags it can be configured, what DetectNet should return.
 * box,labels,conf means bounding boxes, labels (which object was detected), and the confidence.
 * *************************************************************************************************/
    detectNet* net;
    commandLine cmdLine(argc, argv, (const char*)NULL);
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("box,labels,conf");
    detectNet::Detection* detections = NULL;
    int numDetections;
    
/**************************************
 * OpenCV Mat object which is used for:
 * - detecting objects
 * - showing the picture on the screen
 * ************************************/
    cv::Mat image;
    
#ifdef CLOCK_RUNTIME
/******************************************************************
 * Variables used for showing the runtime needed by complete cycle, 
 * grabbing the frames, detecting the objects, 
 * calculating and drawing the coordinates to the image 
 * and filtering the depth frame using the clock
 * ****************************************************************/
    timeval totalStartFPS, totalStartFRAME, totalStartOBJDET;
    #ifdef FILTER
    timeval totalStartFILT;
    #endif
#endif

#if defined(USESHM) || defined(GPIO_RUNTIME) || defined(CLOCK_RUNTIME)
/*****************************************************************
 * Variable used for storing the return value of several functions 
 * ***************************************************************/
    int ret;
#endif
    
#ifdef SAVEPIC
/********************************************************************
 * Counter used for limiting the loop when pictures are saved on disk
 * ******************************************************************/
    int counter = 0;
#endif

#ifdef SHOW_DEPTH
/**********************************************************************************************************************
 * Variables and objects used for coloring the depth map for visualization
 * rs2::colorizer can be configured by setting the option RS2_OPTION_COLOR_SCHEME.
 * This option configures the used colormap. Default is Jet Colormap (0.f).
 * All available option values can be found in rs_processing.hpp from Librealsense in "class colorizer : public filter"
 * ********************************************************************************************************************/
    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, COLORSCHEME);
#endif

#ifdef GPIO_RUNTIME
    #ifdef LOGCONSOLE
    std::cout << "Initializing GPIO..." << std::endl;
    #endif
/********************************************************************
 * Initialization of the GPIOs.
 * The defined pins are exported, their direction is set to Output
 * and their value is set to 0.
 * In case of error, the error message is printed in the subfunctions
 * and the application exits.
 * ******************************************************************/
    ret = gpio_init();
    if(ret < 0)
    {
#if defined(USESHM)
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, -1, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, 1);
#endif
        return EXIT_FAILURE;
    }

#endif

#ifdef LOGCONSOLE
    std::cout << "Initializing Camera..." << std::endl;
#endif
/************************************************************
 * Initialization of the camera.
 * In this function, the depth and color frames are enabled
 * with the defined resolution and a framerate of 30 FPS.
 * The resolution can be configured in "global_defines.h".
 * **********************************************************/
    camera_init(&cfg, &p, &prof);
    
#ifdef LOGCONSOLE
    std::cout << "Loading Camera Intrinsics..." << std::endl;
#endif
/**********************************************************************
 * Loading the camera intrinsics of the color frame.
 * The intrinsics are needed for the calculation of the 3D coordinates.
 * ********************************************************************/
    const rs2_intrinsics color_intrin = get_camera_intrinsics(&frames, &p);

#ifdef FILTER
    #ifdef LOGCONSOLE
    std::cout << "Initializing filters..." << std::endl;
    #endif
/*********************************************************************
 * Initialization of postprocessing filters for the depth frame.
 * The values are defined in camera.h. 
 * The default values are the default values used in realsense-viewer.
 * *******************************************************************/
    filters_init(&dec_filter, &thr_filter, &spat_filter, &temp_filter);
#endif
    
#ifdef MAXLASERPWR
    #ifdef LOGCONSOLE
    std::cout << "Setting Laser Power to maximum value..." << std::endl;
    #endif
/****************************************************************
 * Setting the power of the infrared laser to its maximum value.
 * The default setting in Librealsense is half the power (180mW).
 * This option increases the power consumption, 
 * but it also increases the quality of the depth frame.
 * **************************************************************/
    set_max_laser_pwr(&prof);

#endif 

#ifdef LOGCONSOLE
    std::cout << "Initializing DetectNet..." << std::endl;
#endif
/*******************************************************************
 * Initialization of DetectNet with the given attributes in cmdLine.
 * The attributes for cmdLine are described above.
 * *****************************************************************/
    net = detectNet::Create(cmdLine);
    if(!net)
    {
        std::cerr << "detectNet: failed to load detectNet model" << std::endl;
#if defined(USESHM) && defined(GPIO_RUNTIME)
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, -1, 1);
#elif defined(USESHM) && !defined(GPIO_RUNTIME)
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, -1);
#elif defined(GPIO_RUNTIME) && !defined(USESHM)
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p);
#endif
        return EXIT_FAILURE;
    }
    
#ifdef USESHM
/***********************************************************************************************
 * Here the shared memory is initialized and mapped to the local XYZ variable.
 * Also, the named semaphore is opened.
 * CAUTION: the name of the semaphore needs to be equal to the name in the read_shm application.
 * *********************************************************************************************/
 #ifdef LOGCONSOLE
    std::cout << "Creating shared memory and semaphore..." << std::endl;
#endif
    // check for a valid key
    if(KEY_RS == -1)
    {
        fprintf(stderr, "invalid key\n");
#ifdef GPIO_RUNTIME            
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, -1, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, -1);
#endif
        return EXIT_FAILURE;
    }
    key_t key = KEY_RS;
    int shmid = shmget(key, sizeof(XYZ)*MAXOBJECTS, 0666|IPC_CREAT);
    if(shmid == -1)
    {
        perror("shmget");
#ifdef GPIO_RUNTIME            
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, shmid, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, shmid);
#endif
        return EXIT_FAILURE;
    }
    XYZ *coordinates = (XYZ*) shmat(shmid, (void*)0, 0);
    if(coordinates == (void*)-1)
    {
        perror("shmat");
#ifdef GPIO_RUNTIME            
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, shmid, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, NULL, shmid);
#endif
        return EXIT_FAILURE;
    }
    sem_t* semptr = sem_open("sem_rs", O_CREAT, 0666, 0);
    if(semptr == SEM_FAILED)
    {
        perror("sem_open");
#ifdef GPIO_RUNTIME            
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, coordinates, shmid, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, NULL, coordinates, shmid);
#endif        
        return EXIT_FAILURE;
    }
    if(sem_post(semptr) == -1)
    {
#ifdef GPIO_RUNTIME            
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, semptr, coordinates, shmid, 1);
#else
        cleanup(imgBufferRGB, imgBufferRGBAf, &p, semptr, coordinates, shmid);
#endif        
        return EXIT_FAILURE;
    }

#endif
    
/***************************************************
 * Checks if a SIGINT signal was already received,
 * if not, the camera and detection loop is started.
 * *************************************************/
#ifdef LOGCONSOLE
    if(flag == 0)
    {
        std::cout << "Starting camera and detection loop..." << std::endl;
    }
#endif

/***********************************************************************************************************************************
 * This is an infinite loop that has the following tasks:
 * - Grabbing the color and depth frame
 * - If configured, applying post-processing filters on the depth frame
 * - Detecting objects in the color frame
 * - Calculating the 3D coordinates of the detected objects and sorting them by distance, ascending
 * - If configured, drawing the bounding boxes of the objects to the image and visualizing it
 * - If configured, visualizing the depth frame
 * - If configured, the 3D coordinates are written to a shared memory in the object detection functions,
 *   or, if no object was found some default values are written to the shared memory by the main function. 
 * 
 * The application exits, if CTRL-C is hit in the command window (this raises a SIGINT signal).
 * In case #SHOW_IM or #SHOW_DEPTH is defined, the loop also ends if ESC is hit in the window that shows the image or the depth map.
 * In case #SAVEPIC is defined, the loop ends after 1000 cycles.
 * *********************************************************************************************************************************/
    while(flag == 0)
    { 
        
#ifdef CLOCK_RUNTIME
    #ifdef LOGCONSOLE
        std::cout << "Loading current time for FPS and FRAME..." << std::endl;
    #endif
/*****************************************************************************************
 * gettimeofday loads the current time since the Epoch.
 * The structs hold the time in seconds and microseconds.
 * The values obtained here are used for calculating the time needed for a complete cycle,
 * and for grabbing the frames.
 * ***************************************************************************************/
        ret = gettimeofday(&totalStartFPS, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not get current start FPS time..." << std::endl;
        }
        ret = gettimeofday(&totalStartFRAME, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not get current start FRAME time..." << std::endl;
        }
#endif
        
#ifdef GPIO_RUNTIME
/************************************************************************************************************
 * sets the value of PIN_FPS and PIN_FRAME to 1 to signal the start of a cycle and obtaining a frame via gpio
 * **********************************************************************************************************/
        ret = gpio_set_value(PIN_FPS, 1);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FPS to 1..." << std::endl;
        }
        ret = gpio_set_value(PIN_FRAME, 1);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FRAME to 1..." << std::endl;
        }
#endif
        
#ifdef LOGCONSOLE
        std::cout << "Loading current frames..." << std::endl;
#endif
/************************************************************************************
 * In get_frames the color and depth frame are grabbed.
 * The function returns the depth frame and converts the color frame to a OpenCV Mat.
 * The color frame is saved in the Mat "image".
 * **********************************************************************************/
        rs2::depth_frame depth = get_frames(&frames, &p, &align_to_color, &image);

#ifdef GPIO_RUNTIME
/**********************************************************************************
 * sets the value of PIN_FRAME to 0 to signal the end of obtaining a frame via gpio
 * ********************************************************************************/
        ret = gpio_set_value(PIN_FRAME, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FRAME to 0..." << std::endl;
        }
#endif
        
#ifdef CLOCK_RUNTIME
/******************************************************************
 * Here the current time is loaded and the time difference 
 * to the time value before "get_frames" is calculated and printed.
 * ****************************************************************/
        print_time(&totalStartFRAME, "Time for grabbing the frames: ");        
#endif
        
#ifdef FILTER
/**************************************************************************************************************
 * In this part, the filtering of the depth frame takes place.
 * In case #CLOCK_RUNTIME is defined, also the time is measured how long the filtering process takes. 
 * In case #GPIO_RUNTIME is defined, also the gpio PIN_FILTER is set to 1 and 0 to output the runtime via gpio.
 * ************************************************************************************************************/
    #ifdef CLOCK_RUNTIME
        #ifdef LOGCONSOLE
        std::cout << "Loading current time for FILTER..." << std::endl;
        #endif
        ret = gettimeofday(&totalStartFILT, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not get current start FILTER time..." << std::endl;
        }
    #endif
        
    #ifdef GPIO_RUNTIME
        ret = gpio_set_value(PIN_FILTER, 1);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FILTER to 1..." << std::endl;
        }
    #endif
        
    #ifdef LOGCONSOLE
        std::cout << "Start filtering the depth map..." << std::endl;
    #endif
        
        rs2::depth_frame filtered = filter_depth_frame(&depth, &dec_filter, &thr_filter, &depth_to_disparity, &spat_filter, &temp_filter, &disparity_to_depth, &align_to_color);   

    #ifdef GPIO_RUNTIME
        ret = gpio_set_value(PIN_FILTER, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FILTER to 0..." << std::endl;
        }
    #endif
        
    #ifdef CLOCK_RUNTIME
        print_time(&totalStartFILT, "Time for filtering the depth frame: ");
    #endif
#endif
    
#ifdef COPTER
/*****************************************************************************************
 * On the copter, the camera is rotated by 180° and so the image also needs to be rotated.
 * This is essential for the visualization, but also for the object detection.
 * ***************************************************************************************/
    #ifdef LOGCONSOLE
        std::cout << "Rotating image for the copter..." << std::endl;
    #endif
        rotate(image, image, 1);

#endif
        
/****************************************************************************************************************
 * Here the function for detecting objects in the color image is called.
 * It converts the OpenCV Mat to a float4 array and loads it to the GPU,
 * as it is needed in this format by DetectNet.
 * Then the detection with DetectNet is done and the found objects are returned with the configured overlayFlags. 
 * In case #CLOCK_RUNTIME is defined, also the time is measured how long the object detection takes. 
 * In case #GPIO_RUNTIME is defined, also the gpio PIN_OBJDET is set to 1 and 0 to output the runtime via gpio.
 * **************************************************************************************************************/
#ifdef CLOCK_RUNTIME
    #ifdef LOGCONSOLE
        std::cout << "Loading current time for OBJDET..." << std::endl;
    #endif
        ret = gettimeofday(&totalStartOBJDET, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not get current start OBJDET time..." << std::endl;
        }
#endif
        
#ifdef GPIO_RUNTIME
        ret = gpio_set_value(PIN_OBJDET, 1);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO OBJDET to 1..." << std::endl;
        }
#endif
        
#ifdef LOGCONSOLE
        std::cout << "Starting object detection..." << std::endl;
#endif
        // running object detection
        detections = objDet_run(&numDetections, net, &image, overlayFlags, imgBufferRGB, imgBufferRGBAf);

#ifdef GPIO_RUNTIME
        ret = gpio_set_value(PIN_OBJDET, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO OBJDET to 0..." << std::endl;
        }
#endif
        
#ifdef CLOCK_RUNTIME
/******************************************************************
 * Here the current time is loaded and the time difference 
 * to the time value before "objDet_run" is calculated and printed.
 * ****************************************************************/
        print_time(&totalStartOBJDET, "Time for object detection: ");
#endif
        
/********************************************************************************************************
 * If DetectNet found 1 or more objects in the color image,
 * the depth and the 3D coordinates are calculated in objDet_calc_coordinates.
 * If #SHOW_IM is defined, the bounding boxes and the coordinates are drawn to the color image.
 * In case of active filters, the filtered depth frame is used for getting the depth.
 * In case the coordinates should be written to a shared memory, this is done in objDet_calc_coordinates.
 * If no object was found, some default values are written to the shared memory by the main function.
 * ******************************************************************************************************/
        if(numDetections > 0)
        {
#ifdef LOGCONSOLE
            std::cout << "Starting calculation of coordinates..." << std::endl;
#endif
            
#ifdef USESHM
    #ifdef FILTER
            objDet_calc_coordinates(detections, numDetections, net, &filtered, color_intrin, &image, coordinates, semptr);
    #else
            objDet_calc_coordinates(detections, numDetections, net, &depth, color_intrin, &image, coordinates, semptr);
    #endif
#else
    #ifdef FILTER
            objDet_calc_coordinates(detections, numDetections, net, &filtered, color_intrin, &image);
    #else
            objDet_calc_coordinates(detections, numDetections, net, &depth, color_intrin, &image);
    #endif        
#endif
        }

#ifdef USESHM
        else
        {
			if(sem_wait(semptr) == 0)
			{
				for(int n=0; n < MAXOBJECTS; n++)
				{
					std::string tempStr = "None";
					strcpy(coordinates[n].objType, tempStr.c_str());
					coordinates[n].x_coord = DEFAULT_X_COORD;
					coordinates[n].y_coord = DEFAULT_Y_COORD;
					coordinates[n].z_coord = DEFAULT_Z_COORD;
					coordinates[n].distance = DEFAULT_DISTANCE;
				}
				sem_post(semptr);
			}
        }
#endif

#ifdef SHOW_IM
/********************************************************
 * cv::imshow creates a window and visualizes color image
 * ******************************************************/
    #ifdef LOGCONSOLE
        std::cout << "Showing color image..." << std::endl;
    #endif
        cv::imshow("RealSense Image", image);
#endif
        
#ifdef SHOW_DEPTH
/***************************************************************************
 * In this part, the depth frames are colored using a rs2::colorizer object.
 * To visualize it, the frames need to be converted to an OpenCV Mat.
 * Once done, they are visualized with cv::imshow.
 * *************************************************************************/
    #ifdef LOGCONSOLE
        std::cout << "Showing depth map..." << std::endl;
    #endif
        
        show_depth_frame(&color_map, &depth, "");
    
    #ifdef FILTER 
        #ifdef LOGCONSOLE
        std::cout << "Showing filtered depth map..." << std::endl;
        #endif
        
        show_depth_frame(&color_map, &filtered, "filtered");
    #endif    
#endif

#ifdef SAVEPIC
/****************************************************************************
 * In this part, the color image is saved in the defined location.
 * The name of the saved images includes a number that counts up.
 * The location and prefix and the format are defined in global_defines.h.
 * **************************************************************************/
    #ifdef LOGCONSOLE
        std::cout << "Saving color image to disk..." << std::endl;
    #endif
        std::string name = PICLOCATIONANDPREFIX + std::to_string(counter) + PICFORMAT;
        cv::imwrite(name, image);
        counter++;
        if(counter > 999)
            break;
#endif
     
#ifdef GPIO_RUNTIME
/***********************************************************************
 * Sets the value of PIN_FPS to 0 to signal the end of a cycle via gpio.
 * *********************************************************************/
        ret = gpio_set_value(PIN_FPS, 0);
        if(ret < 0)
        {
            // not a critical error, no further action needed.
            std::cerr << "Could not set GPIO FPS to 0..." << std::endl;
        }
#endif
        
#ifdef CLOCK_RUNTIME
/******************************************************************
 * Here the current time is loaded and the time difference 
 * to beginning of the cycle is calculated and printed.
 * ****************************************************************/
        print_time(&totalStartFPS, "Total time: ");
#endif
        
#if defined(LOGCONSOLE) || defined(LOGOBJECTS) || defined(CLOCK_RUNTIME)
        std::cout << std::endl;
#endif
        
#if defined(SHOW_IM) || defined(SHOW_DEPTH)
/******************************************************************************************************
 * cv::waitKey is needed by cv::imshow.
 * In this case it listens for 1ms if a key is pressed while a window created by cv::imshow is focused.
 * If the key was ESC, the infinite while loop ends.
 * ****************************************************************************************************/
        int keycode = cv::waitKey(1) & 0xff;
        if(keycode == 27)
        {
            break;
        }
#endif
        
    }

/********************************************************
 * Cleaning up used resources and exiting the application
 * ******************************************************/
#ifdef USESHM
    #ifdef GPIO_RUNTIME            
        retval = cleanup(imgBufferRGB, imgBufferRGBAf, &p, semptr, coordinates, shmid, 1);
    #else
        retval = cleanup(imgBufferRGB, imgBufferRGBAf, &p, semptr, coordinates, shmid);
    #endif  
#else
    #ifdef GPIO_RUNTIME            
        retval = cleanup(imgBufferRGB, imgBufferRGBAf, &p, 1);
    #else
        retval = cleanup(imgBufferRGB, imgBufferRGBAf, &p);
    #endif  
#endif
    
#ifdef LOGCONSOLE
    std::cout << "Exiting now..." << std::endl;
#endif

    return retval;
}
