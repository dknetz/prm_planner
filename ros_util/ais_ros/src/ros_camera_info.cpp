/*
 * ros_camera_info.cpp
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <ais_ros/ros_camera_info.h>

namespace ais_ros
{

RosCameraInfo::RosCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo) :
				ais_point_cloud::RGBDImage::CameraInfo(
						camInfo->P[2],
						camInfo->P[6],
						1.0 / camInfo->P[0],
						1.0 / camInfo->P[5],
						camInfo->width,
						camInfo->height),
				m_camInfo(camInfo)
{
}

RosCameraInfo::~RosCameraInfo()
{
}

} /* namespace ais_ros */
