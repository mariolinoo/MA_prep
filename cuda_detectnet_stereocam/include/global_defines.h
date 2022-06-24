#ifndef __GLOBAL_DEFINES__H__
#define __GLOBAL_DEFINES__H__

/****************************** Global defines *********************************************************************************
 * #SHOW_IM --> Show images and boxes for the detected objects
 * #SHOW_DEPTH --> Show colorized depth map, in case #FILTER is active, both depth maps are shown
 * #LOGCONSOLE --> enable advanced logging in the console
 * #LOGOBJECTS --> enable logging for the objects in the console
 * #FILTER --> applies post-processing filters to depth map, only used with #REALSENSE
 * #MAXLASERPWR --> sets the power of the laser to maximum, only used with #REALSENSE
 * #COPTER --> Rotate image by 180° if the camera is mounted like this on the copter
 * #GPIO_RUNTIME --> used for GPIO output for runtime measurements
 * #CLOCK_RUNTIME --> used for runtime measurements with the sysclock
 * #USESHM --> coordinates should be written to shared memory
 * #SAVEPIC --> saves every frame as JPG in folder ./pics, ends after 1000 frames
 * #RESOLUTION --> Resolution for Realsense camera, not used for Steereocam: 1 -> 1280x720, 2 -> 848x480, 3 or anything else -> 424x240
 * #MAXOBJECTS --> defines the maximum number of objects for 3D calculation
 *          CAUTION: In case #USESHM is defined, this should have the same value as in the read_shm application,
 *                  otherwise objects may be lost or an invalid memory access occurs.
 * #REALSENSE --> Realsense target, must not be used together with #STEEREOCAM
 * #STEEREOCAM --> Steereocam target, must not be used together with #REALSENSE
 * 
 * CAUTION: #MAXOBJECTS needs to have the same value as in the read_shm application. Otherwise objects may be lost or an invalid memory access occurs.
 * *****************************************************************************************************************************/
#define SHOW_IM
#define SHOW_DEPTH
#define LOGCONSOLE
#define LOGOBJECTS
#define FILTER
#define MAXLASERPWR
#define COPTER
//#define GPIO_RUNTIME
#define CLOCK_RUNTIME
#define USESHM
//#define SAVEPIC
#define RESOLUTION 2
#define MAXOBJECTS 5
#define REALSENSE
//#define STEEREOCAM

#if defined(REALSENSE) && defined(STEEREOCAM)
#error "Realsense and Steereocam defined, please define only one"
#endif

#if defined(STEEREOCAM) && (defined(FILTER) || defined(MAXLASERPWR) || defined(RESOLUTION))
#warning "Steereocam does not support #FILTER, #MAXLASERPWR and #RESOLUTION, they will be ignored"
#endif


/********************************************************************
 * In case the 3D coordinates of a found object cannot be calculated,
 * the below default coordinates and distance will be applied.
 * ******************************************************************/
#define DEFAULT_X_COORD -1.0f
#define DEFAULT_Y_COORD -1.0f
#define DEFAULT_Z_COORD -1.0f
#define DEFAULT_DISTANCE -1.0f

/************************************************
 * Possible resolutions for the realsense camera.
 * CAUTION: do not change these values!!
 * **********************************************/
#ifdef REALSENSE
    #if RESOLUTION == 1
        #define WIDTH 1280
        #define HEIGHT 720
        #define FRAMERATE 30
    #elif RESOLUTION == 2
        #define WIDTH 848
        #define HEIGHT 480
        #define FRAMERATE 30
    #else
        #define WIDTH 424
        #define HEIGHT 240
        #define FRAMERATE 30
    #endif
#endif

/************************************************
 * Possible resolutions for the steereocam camera.
 * CAUTION: do not change these values!!
 * **********************************************/
#ifdef STEEREOCAM
    #define WIDTH 1600
    #define HEIGHT 1300
#endif

/*******************************************************************************************************
 * These defines define the pins that should be used if the runtime shall be measured via GPIO toggling.
 * *****************************************************************************************************/
#ifdef GPIO_RUNTIME
    #define PIN_FPS 79
    #define PIN_OBJDET 50
    #define PIN_FRAME 216
    #if defined(FILTER) && defined(REALSENSE)
        #define PIN_FILTER 14
    #endif
#endif

/******************************************************************************
 * Here the key for the shared memory is defined.
 * CAUTION: this should be equal to the key in the read_shm application.
 * ****************************************************************************/
#ifdef USESHM
	#ifdef REALSENSE
    	#define KEY_RS 1234
	#else
		#define KEY_SC 2345
	#endif
#endif

/*******************************************************************
 * These defines are for the location 
 * where the images shall be saved with their prefix and the format.
 * *****************************************************************/
#ifdef SAVEPIC
    #define PICLOCATIONANDPREFIX "./pics/Pic_"
    #define PICFORMAT ".jpg"
#endif

/***************************************************************
 * Here the colorscheme is defined.
 * 0.f means JET colorscheme.
 * All available option values can be found in rs_processing.hpp 
 * from Librealsense in "class colorizer : public filter"
 * *************************************************************/
#if defined (SHOW_DEPTH) && defined(REALSENSE)
    #define COLORSCHEME 0.f
#endif


/***********************************************************************************
 * Here the configuration of the parameters for the post-processing filters is done.
 * The default values for post-processing were taken from realsense-viewer.
 * If custom values should be used, please change the defines from DEFAULT_FILTER 
 * to CUSTOM_FILTER and adapt the values under the #ifdef CUSTOM_FILTER
 * *********************************************************************************/
#ifdef FILTER
    #define DEFAULT_FILTER
    //#define CUSTOM_FILTER
    #ifdef DEFAULT_FILTER
        #define DEC_MAGN 2.0f
        #define THR_MINDIST 0.1f
        #define THR_MAXDIST 8.1f
        #define SPAT_MAGN 2.0f
        #define SPAT_ALPH 0.5f
        #define SPAT_DELT 20.0f
        #define TEMP_ALPH 0.71f
        #define TEMP_DELT 61.0f
    #endif
    #ifdef CUSTOM_FILTER
        #define DEC_MAGN 2.0f
        #define THR_MINDIST 0.1f
        #define THR_MAXDIST 8.1f
        #define SPAT_MAGN 2.0f
        #define SPAT_ALPH 0.5f
        #define SPAT_DELT 20.0f
        #define TEMP_ALPH 0.71f
        #define TEMP_DELT 61.0f
    #endif
#endif

/*****************************************************************************
 * This structure is used for holding the coordinates of the detected objects.
 * ***************************************************************************/
typedef struct XYZ
{
    char objType[20];
    float x_coord;
    float y_coord;
    float z_coord;
    float distance;
} XYZ;

#endif
