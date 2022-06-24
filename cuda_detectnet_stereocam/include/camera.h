#ifndef __CAMERA__H__
#define __CAMERA__H__

#include "global_defines.h"
#include <librealsense2/rs.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>

#ifdef SHOW_DEPTH
#include <opencv2/highgui.hpp>
#endif

/**********************************************************************************************
 * Function prototypes for realsense camera functions, the functions are described in camera.cu
 * ********************************************************************************************/
void camera_init(rs2::config *cfg, rs2::pipeline *p, rs2::pipeline_profile *prof);
rs2_intrinsics get_camera_intrinsics(rs2::frameset *frames, rs2::pipeline *p);
rs2::depth_frame get_frames(rs2::frameset *frames, rs2::pipeline *p, rs2::align *align_to_color, cv::Mat *image);

#ifdef FILTER
    void filters_init(rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter);
    rs2::depth_frame filter_depth_frame(rs2::depth_frame *depth, rs2::decimation_filter *dec_filter, rs2::threshold_filter *thr_filter, rs2::disparity_transform *depth_to_disparity, rs2::spatial_filter *spat_filter, rs2::temporal_filter *temp_filter, rs2::disparity_transform *disparity_to_depth, rs2::align *align_to_color);
#endif

#ifdef MAXLASERPWR
    void set_max_laser_pwr(rs2::pipeline_profile *prof);
#endif
    
#ifdef SHOW_DEPTH
    void show_depth_frame(rs2::colorizer *color_map, rs2::depth_frame *depth, std::string text);
#endif

#endif
