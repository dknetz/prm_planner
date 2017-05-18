/*
 * This file (ros_base_interface.cpp) is part of the "ais_ros"-package of Daniel Kuhner.
 *
 * It is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
 *
 * created:		Dec 19, 2014
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#include <cv_bridge/cv_bridge.h>
#include <ais_definitions/macros.h>
#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <ais_ros/ros_base_interface.h>
#include <ais_ros/ros_camera_info.h>
#include <ais_ros/ros_time.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv/highgui.h>
#else
#include <opencv2/highgui.hpp>
#endif

using namespace ros;
using namespace ais_util;
using namespace ais_point_cloud;
using namespace ais_definitions;

namespace ais_ros
{

tf::TransformListener* RosBaseInterface::m_listener = NULL;

RosBaseInterface::RosBaseInterface() :
				depthAndRGBImageCallbackActive(false),
				depthCallbackActive(false),
				rgbCallbackActive(false),
				depthCameraInfoCallbackActive(false),
				rgbCameraInfoCallbackActive(false),
				spinner(NULL),
				syncedDepthSub(NULL),
				syncedColorSub(NULL),
				syncronizer(NULL),
				camInfoDepthReceived(false),
				camInfoRGBReceived(false),
				m_mirrorImages(false)
{
}

RosBaseInterface::~RosBaseInterface()
{
	deactivateAllCallbacks();
	DELETE_VAR(spinner);
//	DELETE_VAR(syncedDepthSub);
//	DELETE_VAR(syncedColorSub);
//	DELETE_VAR(syncronizer);
}

void RosBaseInterface::startRosBaseInterface(const bool startAsyncSpinner)
{
	NodeHandle n;

	if (startAsyncSpinner && spinner == NULL)
	{
		spinner = new ros::AsyncSpinner(0);
		spinner->start();
	}
}

//CALLBACKS: NEED TO BE IMPLEMENTED IN A SUBCLASS
void RosBaseInterface::callbackRGBDImage(RGBDImage::Ptr& image)
{
}

void RosBaseInterface::callbackRGBImage(cv::Mat& image)
{
}

void RosBaseInterface::getRGBCameraInfo(sensor_msgs::CameraInfo::Ptr& cameraInfo)
{
	cameraInfo = camInfoRGB;
}

void RosBaseInterface::getDepthCameraInfo(sensor_msgs::CameraInfo::Ptr& cameraInfo)
{
	cameraInfo = camInfoDepth;
}

void RosBaseInterface::waitUntilCameraInfoIsAvailable()
{
	if (depthCameraInfoCallbackActive)
	{
		while (camInfoDepth.get() == NULL && ros::ok())
		{
			usleep(10000);
		}
	}

	if (rgbCameraInfoCallbackActive)
	{
		while (camInfoRGB.get() == NULL && ros::ok())
		{
			usleep(10000);
		}
	}
}

//ACTIVATORS: ACTIVATE THE CALLBACKS
void RosBaseInterface::useRGBDImages(const std::string& topicRGBImage,
		const std::string& topicDepthImage,
		const std::string& topicRGBCameraInfo,
		const std::string& topicDepthCameraInfo)
{
	activeCallbacks[ROS_BASE_INTERFACE_CALLBACK_RGBD_IMAGE] = true;
	activateRGBCameraInfo(topicRGBCameraInfo);
	activateDepthCameraInfo(topicDepthCameraInfo);
	activateDepthAndRGBCamera(topicRGBImage, topicDepthImage);
}

void RosBaseInterface::useRGBImage(const std::string& topicRGBImage,
		const std::string& topicRGBCameraInfo)
{
	activeCallbacks[ROS_BASE_INTERFACE_CALLBACK_RGB_IMAGE] = true;
	activateRGBCameraInfo(topicRGBCameraInfo);
	activateRGBCamera(topicRGBImage);
}

void RosBaseInterface::useDepthImage(const std::string& topicDepthImage,
		const std::string& topicDepthCameraInfo)
{
	activeCallbacks[ROS_BASE_INTERFACE_CALLBACK_DEPTH_IMAGE] = true;
	activateDepthCameraInfo(topicDepthCameraInfo);
	activateDepthCamera(topicDepthImage);
}

//CALLBACKS: ACTUAL CALLBACKS
void RosBaseInterface::rosCallbackRGBD(const sensor_msgs::Image::ConstPtr& depthImage,
		const sensor_msgs::Image::ConstPtr& rgbImage)
{
	sensor_msgs::CameraInfo::Ptr camInfoDepth;
	sensor_msgs::CameraInfo::Ptr camInfoRGB;

	getDepthCameraInfo(camInfoDepth);
	getRGBCameraInfo(camInfoRGB);

	cv_bridge::CvImagePtr depthCV;
	cv_bridge::CvImagePtr colorCV;

	try
	{
		if (rgbImage->encoding == sensor_msgs::image_encodings::RGB8)
		{
			colorCV = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::RGB8);
		}
		else
		{
			colorCV = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::RGB8);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		LOG_ERROR("cv_bridge exception: " << e.what());
	}

	try
	{
		depthCV = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		LOG_ERROR("cv_bridge exception: " << e.what());
	}

	if (camInfoDepthReceived)
	{
		if (CHECK_MAP(activeCallbacks, ROS_BASE_INTERFACE_CALLBACK_RGBD_IMAGE))
		{
			RGBDImage::Ptr image(new RGBDImage(colorCV->image, depthCV->image, ais_ros::RosCameraInfo(camInfoDepth)));
			image->setMirrorImages(m_mirrorImages);
			image->m_time = ais_ros::RosTime(depthImage->header.stamp);

			//we transform the point cloud directly into parent frame
			if (!m_cameraFrameName.empty() && !m_cameraParentFrameName.empty())
			{
				image->setFrame(m_cameraParentFrameName);
				Eigen::Affine3d t;
				if (!getRosTransformationWithResult(m_cameraFrameName, m_cameraParentFrameName, t))
					return;
				image->setTransformation(t);
			}
			else if (!m_cameraFrameName.empty())
			{
				image->setFrame(m_cameraFrameName);
			}

			callbackRGBDImage(image);
		}
		//else wait until we got the info
	}
}

void RosBaseInterface::rosCallbackDepthOnly(const sensor_msgs::Image::ConstPtr& depthImage)
{
	cv_bridge::CvImagePtr depthCV;

	try
	{
		depthCV = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		LOG_ERROR("cv_bridge exception: " << e.what());
	}

	if (camInfoDepthReceived)
	{
		if (CHECK_MAP(activeCallbacks, ROS_BASE_INTERFACE_CALLBACK_DEPTH_IMAGE))
		{
			RGBDImage::Ptr image(new RGBDImage(depthCV->image, ais_ros::RosCameraInfo(camInfoDepth)));
			image->setMirrorImages(m_mirrorImages);
			image->m_time = ais_ros::RosTime(depthImage->header.stamp);

			//we transform the point cloud directly into parent frame
			if (!m_cameraFrameName.empty() && !m_cameraParentFrameName.empty())
			{
				image->setFrame(m_cameraParentFrameName);
				Eigen::Affine3d t;
				if (!getRosTransformationWithResult(m_cameraFrameName, m_cameraParentFrameName, t))
					return;

				image->setTransformation(t);
			}
			else if (!m_cameraFrameName.empty())
			{
				image->setFrame(m_cameraFrameName);
			}

			callbackRGBDImage(image);
		}
	}
}

void RosBaseInterface::rosCallbackRGBOnly(const sensor_msgs::Image::ConstPtr& rgbImage)
{
	sensor_msgs::CameraInfo::Ptr camInfoRGB;
	getRGBCameraInfo(camInfoRGB);
	cv_bridge::CvImagePtr colorCV;

	try
	{
		if (rgbImage->encoding == sensor_msgs::image_encodings::RGB8)
		{
			colorCV = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::RGB8);
		}
		else
		{
			colorCV = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		LOG_ERROR("cv_bridge exception: " << e.what());
	}

	if (CHECK_MAP(activeCallbacks, ROS_BASE_INTERFACE_CALLBACK_RGB_IMAGE))
	{
		callbackRGBImage(colorCV->image);
	}
}

void RosBaseInterface::rosCallbackDepthCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo)
{
	camInfoDepth = camInfo;
	camInfoDepthReceived = true;
	subCameraInfoDepth.shutdown();
	depthAndRGBImageCallbackActive = false;
}

void RosBaseInterface::rosCallbackRGBCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo)
{
	camInfoRGB = camInfo;
	camInfoRGBReceived = true;
	subCameraInfoRGB.shutdown();
	rgbCameraInfoCallbackActive = false;
}

void RosBaseInterface::activateDepthAndRGBCamera(const std::string& topicRGBImage,
		const std::string& topicDepthImage)
{
	if (!depthAndRGBImageCallbackActive)
	{
		NodeHandle n;

		syncedDepthSub = new message_filters::Subscriber<sensor_msgs::Image>(n, topicDepthImage, 1);
		syncedColorSub = new message_filters::Subscriber<sensor_msgs::Image>(n, topicRGBImage, 1);
		syncronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *syncedDepthSub, *syncedColorSub);
		syncronizer->registerCallback(boost::bind(&RosBaseInterface::rosCallbackRGBD, this, _1, _2));

		depthAndRGBImageCallbackActive = true;
	}
}

void RosBaseInterface::activateDepthCamera(const std::string& topicDepthImage)
{
	if (!depthCallbackActive)
	{
		NodeHandle n;
		subCameraDepth = n.subscribe(topicDepthImage, 1, &RosBaseInterface::rosCallbackDepthOnly, this);
		depthCallbackActive = true;
	}
}

void RosBaseInterface::activateRGBCamera(const std::string& topicRGBImage)
{
	if (!rgbCallbackActive)
	{
		NodeHandle n;
		subCameraRGB = n.subscribe(topicRGBImage, 1, &RosBaseInterface::rosCallbackRGBOnly, this);
		rgbCallbackActive = true;
	}
}

void RosBaseInterface::activateRGBCameraInfo(const std::string& topicRGBImageInfo)
{
	if (!rgbCameraInfoCallbackActive)
	{
		NodeHandle n;
		subCameraInfoRGB = n.subscribe(topicRGBImageInfo, 1, &RosBaseInterface::rosCallbackRGBCameraInfo, this);
		rgbCameraInfoCallbackActive = true;
	}
}

void RosBaseInterface::deactivateAllCallbacks()
{
	deactivateImageCallbacks();
	DELETE_VAR(spinner);
}

void RosBaseInterface::deactivateImageCallbacks()
{
	subCameraDepth.shutdown();
	subCameraInfoDepth.shutdown();
	subCameraRGB.shutdown();
	subCameraInfoRGB.shutdown();

	DELETE_VAR(syncronizer);
	DELETE_VAR(syncedDepthSub);
	DELETE_VAR(syncedColorSub);

	depthAndRGBImageCallbackActive = false;
	depthCallbackActive = false;
	rgbCallbackActive = false;
	depthCameraInfoCallbackActive = false;
	rgbCameraInfoCallbackActive = false;
}

Eigen::Affine3d RosBaseInterface::getTransformation(const std::string& from,
		const std::string& to,
		ros::Time time)
{
	return RosBaseInterface::getRosTransformation(from, to, time);
}

void RosBaseInterface::initTransformListener()
{
	DELETE_VAR(m_listener);
	m_listener = new tf::TransformListener;
}

Eigen::Affine3d RosBaseInterface::getRosTransformation(const std::string& from,
		const std::string& to,
		ros::Time time)
{
	Eigen::Affine3d res;
	getRosTransformationWithResult(from, to, res, time);
	return res;
}

bool RosBaseInterface::getRosTransformationWithResult(const std::string& from,
		const std::string& to,
		Eigen::Affine3d& t,
		ros::Time time)
{
	if (m_listener == NULL)
	{
		m_listener = new tf::TransformListener;
	}

	tf::StampedTransform transform;
	transform.setIdentity();
	try
	{
		m_listener->lookupTransform(to, from,
				time, transform);
	}
	catch (tf::TransformException& ex)
	{
		return false;
	}

	tf::transformTFToEigen(transform, t);
	return true;
}

void RosBaseInterface::activateDepthCameraInfo(const std::string& topicDepthImageInfo)
{
	if (!depthCameraInfoCallbackActive)
	{
		NodeHandle n;
		subCameraInfoDepth = n.subscribe(topicDepthImageInfo, 1, &RosBaseInterface::rosCallbackDepthCameraInfo, this);
		depthCameraInfoCallbackActive = true;
	}
}

void RosBaseInterface::setFrameNamesOfCamera(const std::string& cameraFrame,
		const std::string& cameraParentFrame)
{
	m_cameraFrameName = cameraFrame;
	m_cameraParentFrameName = cameraParentFrame;
}

bool RosBaseInterface::isMirrorImages() const
{
	return m_mirrorImages;
}

void RosBaseInterface::setMirrorImages(bool mirrorImages)
{
	m_mirrorImages = mirrorImages;
}
} /* namespace ais_ros */

