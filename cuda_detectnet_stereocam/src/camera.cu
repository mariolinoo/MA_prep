#include "camera.h"

/**********************************************************************************************
 * This function initializes the camera.
 * It enables the color and depth streams with the defined resolution and framerate.
 * The resolution and framerate are defined through #RESOLUTION in "global_defines.h".
 * Then it starts the camera by starting the rs2::pipeline with the given rs2::config.
 * 
 * Input parameters:
 *  - rs2::config *cfg: Configuration for the rs2::pipeline.
 *  - rs2::pipeline *p: rs2::pipeline for starting the camera with the given rs2::config.
 *  - rs2::pipeline_profile *prof: Profile that holds the started pipeline after this function.
 * 
 * Return values:
 *  - rs2::pipeline_profile *prof: Profile that holds the started pipeline.
 *          The profile is needed for setting the maximum laser power.
 * ********************************************************************************************/
void camera_init(rs2::config *cfg, rs2::pipeline *p, rs2::pipeline_profile *prof)
{
    cfg->enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FRAMERATE);
    
    // Color format is set to BGR as OpenCV is used for the visualization it OpenCV uses BGR
    cfg->enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FRAMERATE);
    
    *prof = p->start(*cfg);
}

/***************************************************************************************************************
 * This function obtains the camera intrinsics of the color frame.
 * The intrinsics are needed for calculating the 3D coordinates of a pixel in the color frame for a given depth.
 * 
 * Input parameters:
 *  - rs2::frameset *frames: The rs2::frameset that holds the frames after grabbing them from the pipeline.
 *  - rs2::pipeline *p: The rs2::pipeline that interfaces with the camera.
 * 
 * Return values:
 *  - const rs2_intrinsics color_intrin: These are the intrinsic parameters of the color camera.
 * *************************************************************************************************************/
rs2_intrinsics get_camera_intrinsics(rs2::frameset *frames, rs2::pipeline *p)
{
    rs2::stream_profile cprof;
    
    *frames = p->wait_for_frames();
    rs2::video_frame color = frames->get_color_frame();
    (void)frames->get_depth_frame();
    
    cprof = color.get_profile();
    rs2::video_stream_profile cvsprof(cprof);
    const rs2_intrinsics color_intrin = cvsprof.get_intrinsics();
    
    return color_intrin;
}

/******************************************************************************************************************
 * This function grabs the color and depth frames from the camera.
 * For calculating the 3D coordinates for detected objects in the color frame, 
 * it is necessary to align the depth frame to the color frame.
 * This ensures that both frames have the same coordinate system 
 * and the pixels from the color frame can be used for getting the depth in the depth frame for the same position.
 * After grabbing the frames, the color frame is converted to an OpenCV Mat for object detection and visualization.
 * 
 * Input parameters:
 *  - rs2::frameset *frames: The rs2::frameset that holds the frames after grabbing them from the pipeline.
 *  - rs2::pipeline *p: The rs2::pipeline that interfaces with the camera.
 *  - rs2::align *align_to_color: rs2::align object that aligns the color and depth frame.
 *  - cv::Mat *image: OpenCV Mat from the calling function to which the color frame should be copied.
 * 
 * Return values:
 *  - rs2::depth_frame depth: This is the depth frame.
 *  - cv::Mat *image: OpenCV Mat that holds the color image.
 * ****************************************************************************************************************/
rs2::depth_frame get_frames(rs2::frameset *frames, rs2::pipeline *p, rs2::align *align_to_color, cv::Mat *image)
{
    float w, h;
    
/***************************************************
 * Grabbing and aligning the frames from the camera.
 * *************************************************/
    *frames = p->wait_for_frames();
#ifndef FILTER
/****************************************************************************************
 * In "https://github.com/IntelRealSense/librealsense/issues/1207#issuecomment-367718085" 
 * Intel recommends to align the depth frame after filtering to reduce aliasing.
 * Therefore it is done after filtering if filters are used.
 * **************************************************************************************/
    *frames = align_to_color->process(*frames);
#endif
    rs2::video_frame color = frames->get_color_frame();
    rs2::depth_frame depth = frames->get_depth_frame(); 
    
/*************************************************************************************
 * Converting the color frame to an OpenCV Mat for object detection and visualization.
 * ***********************************************************************************/
    w = color.get_width();
    h = color.get_height();
    *image = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

/****************************
 * Returning the depth frame.
 * **************************/
    return depth;
}

/**********************************************************************************
 * This function sets the power of the infrared laser to its maximum value.
 * The default value in librealsense is half the power (180mW).
 * Setting the power to the maximum value increases the power consumption,
 * but it also increases the quality of the depth map.
 * For setting the maximum power, first the rs2::device 
 * and rs2::sensor are obtained from the current pipeline profile.
 * Next it is checked, if the sensor supports the option to change the laser power.
 * If it supports it, the min and max values are obtained and the max value is set.
 * 
 * Input parameters:
 *  - rs2::pipeline_profile *prof: Profile that holds the started pipeline.
 * 
 * Return values:
 *  - None
 * ********************************************************************************/
void set_max_laser_pwr(rs2::pipeline_profile *prof)
{    
/****************************************************
 * Getting the device and depth sensor of the camera.
 * **************************************************/
    rs2::device selected_device = prof->get_device();
    rs2::sensor depth_sensor = selected_device.first<rs2::depth_sensor>();

/*********************************************************
 * If the sensor supports changing the laser power, 
 * the min/max range is obtained and the max value is set.
 * Else, nothing will be changed.
 * *******************************************************/
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Getting min/max range
        rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }
    else
    {
        std::cout << "The sensor does not support the option to change the laser power..." << std::endl;
    }
}

#ifdef FILTER
/***************************************************************************
 * This function initializes the filters with the defined values.
 * The values are defined in "global_defines.h".
 * 
 * Input parameters:
 *  - rs2::decimation_filter *dec_filter: A pointer to the decimation filter
 *  - rs2::threshold_filter *thr_filter: A pointer to the threshold filter
 *  - rs2::spatial_filter *spat_filter: A pointer to the spatial filter
 *  - rs2::temporal_filter *temp_filter: A pointer to the temporal filter
 * 
 * Return values:
 *  - Initialized filter objects.
 * *************************************************************************/
void filters_init(rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter)
{
    dec_filter->set_option(RS2_OPTION_FILTER_MAGNITUDE, DEC_MAGN);        
    thr_filter->set_option(RS2_OPTION_MIN_DISTANCE, THR_MINDIST);
    thr_filter->set_option(RS2_OPTION_MAX_DISTANCE, THR_MAXDIST);    
    spat_filter->set_option(RS2_OPTION_FILTER_MAGNITUDE, SPAT_MAGN);
    spat_filter->set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, SPAT_ALPH);
    spat_filter->set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, SPAT_DELT);    
    temp_filter->set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, TEMP_ALPH);
    temp_filter->set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, TEMP_DELT);
}

/***************************************************************************************************************
 * This function applies post-processing filters on the given depth frame.
 * In "https://dev.intelrealsense.com/docs/depth-post-processing" 
 * Intel recommends to apply the spatial filter and temporal filter in disparity space.
 * Therefore two rs2::disparity_transform objects are used.
 * One for transforming the frame to disparity space.
 * And the other for transforming it back to depth space after spatial and temporal filtering.
 * In "https://github.com/IntelRealSense/librealsense/issues/1207#issuecomment-367718085" 
 * Intel recommends to align the depth frame after filtering to reduce aliasing.
 * 
 * Input parameters:
 *  - rs2::depth_frame *depth: The depth frame that should be filtered.
 *  - rs2::decimation_filter *dec_filter: Decimation filter
 *  - rs2::threshold_filter *thr_filter: Threshold filter
 *  - rs2::disparity_transform *depth_to_disparity: Object to transform the frame from depth to disparity space.
 *  - rs2::spatial_filter *spat_filter: Spatial filter
 *  - rs2::temporal_filter *temp_filter: Temporal Filter
 *  - rs2::disparity_transform *disparity_to_depth: Object to transform the frame from disparity to depth space.
 *  - rs2::align *align_to_color: Object to align the filtered depth frame again to the color frame.
 * 
 * Return values:
 *  - rs2::depth_frame filtered: This is the filtered depth frame.
 * *************************************************************************************************************/
rs2::depth_frame filter_depth_frame(rs2::depth_frame *depth, rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::disparity_transform *depth_to_disparity, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter, rs2::disparity_transform *disparity_to_depth, rs2::align *align_to_color)
{
    rs2::depth_frame filtered = *depth;
    filtered = dec_filter->process(filtered);
    filtered = thr_filter->process(filtered);
    filtered = depth_to_disparity->process(filtered);
    filtered = spat_filter->process(filtered);
    filtered = temp_filter->process(filtered);
    filtered = disparity_to_depth->process(filtered); 
    filtered = align_to_color->process(filtered);
    
    return filtered;
}
#endif

/***************************************************************************************************************
 * This function visualizes the given depth frame.
 * First, it colors it with the given rs2::colorizer (i.e. to Jet colormap).
 * Next, it converts the rs2::depth_frame to an OpenCV Mat.
 * Finally, it visualizes the image on the screen. 
 * 
 * Input parameters:
 *  - rs2::colorizer *color_map: rs2::colorizer object that is used for applying a colorscheme to the depth map.
 *  - rs2::depth_frame *depth: The depth map that should be visualized.
 *  - std::string text: Text for naming the window.
 * 
 * Return values:
 *  - None
 * *************************************************************************************************************/
#ifdef SHOW_DEPTH
void show_depth_frame(rs2::colorizer *color_map, rs2::depth_frame *depth, std::string text)
{
    float w, h;
    
/*****************************
 * Colorizing the depth frame.
 * ***************************/
    rs2::frame depthframe = color_map->process(*depth);
 
/***************************************************
 * Converting the rs2::depth_frame to an OpenCV Mat.
 * *************************************************/
    w = depthframe.as<rs2::video_frame>().get_width();
    h = depthframe.as<rs2::video_frame>().get_height();
    cv::Mat depth_img = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)depthframe.get_data(), cv::Mat::AUTO_STEP);
    
#ifdef COPTER
/*****************************************************************************************
 * On the copter, the camera is rotated by 180° and so the image also needs to be rotated.
 * This is essential for the visualization, but also for the object detection.
 * ***************************************************************************************/
#ifdef LOGCONSOLE
    std::cout << "Rotating depth image for the copter..." << std::endl;
#endif
    rotate(depth_img, depth_img, 1);
#endif

/*************************************
 * Visualizing the created OpenCV Mat.
 * ***********************************/
    cv::imshow(text + " Depth Map", depth_img);
}
#endif
