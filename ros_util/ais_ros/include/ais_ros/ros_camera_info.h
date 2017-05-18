/*
 * ros_camera_info.h
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#ifndef ROS_UTIL_INCLUDE_ROS_UTIL_ROS_CAMERA_INFO_H_
#define ROS_UTIL_INCLUDE_ROS_UTIL_ROS_CAMERA_INFO_H_

#include <ais_definitions/class.h>
#include <ais_point_cloud/rgbd_image.h>

namespace ais_ros
{

struct RosCameraInfo: public ais_point_cloud::RGBDImage::CameraInfo
{
PUBLIC_METHODS:
	RosCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo);
	virtual ~RosCameraInfo();

PUBLIC_VARIABLES:
	const sensor_msgs::CameraInfoPtr m_camInfo;
};

} /* namespace ais_ros */

#endif /* ROS_UTIL_INCLUDE_ROS_UTIL_ROS_CAMERA_INFO_H_ */
