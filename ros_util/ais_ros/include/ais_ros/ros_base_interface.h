/*
 * This file (ros_base_interface.h) is part of the "ais_ros"-package of Daniel Kuhner.
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
#ifndef KUHNERD_LIBS_DK_UTIL_INCLUDE_DK_UTIL_ROS_BASE_INTERFACE_H_
#define KUHNERD_LIBS_DK_UTIL_INCLUDE_DK_UTIL_ROS_BASE_INTERFACE_H_

#include <ais_log/log.h>
#include <ais_point_cloud/rgbd_image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

#include <pluginlib/class_loader.h>

#include <ros/spinner.h>

#define ROS_BASE_INTERFACE_CALLBACK_RGBD_IMAGE "rgbd_images"
#define ROS_BASE_INTERFACE_CALLBACK_RGB_IMAGE "rgb_image"
#define ROS_BASE_INTERFACE_CALLBACK_DEPTH_IMAGE "depth_image"

namespace ais_ros
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class RosBaseInterface
{
public:
	RosBaseInterface();
	virtual ~RosBaseInterface();

	virtual void startRosBaseInterface(const bool startAsyncSpinner = true);

	//callbacks which can be used for getting the data
	virtual void callbackRGBDImage(ais_point_cloud::RGBDImage::Ptr& image);
	virtual void callbackRGBImage(cv::Mat& image);

	virtual void getRGBCameraInfo(sensor_msgs::CameraInfo::Ptr& cameraInfo);
	virtual void getDepthCameraInfo(sensor_msgs::CameraInfo::Ptr& cameraInfo);
	virtual void waitUntilCameraInfoIsAvailable();

	void useRGBDImages(const std::string& topicRGBImage,
			const std::string& topicDepthImage,
			const std::string& topicRGBCameraInfo,
			const std::string& topicDepthCameraInfo);
	void useRGBImage(const std::string& topicRGBImage,
			const std::string& topicRGBCameraInfo);
	void useDepthImage(const std::string& topicDepthImage,
			const std::string& topicDepthCameraInfo);

	virtual void deactivateAllCallbacks();
	virtual void deactivateImageCallbacks();

	virtual Eigen::Affine3d getTransformation(const std::string& from,
			const std::string& to,
			ros::Time time = ros::Time(0));
	static void initTransformListener();
	static Eigen::Affine3d getRosTransformation(const std::string& from,
			const std::string& to,
			ros::Time time = ros::Time(0));
	static bool getRosTransformationWithResult(const std::string& from,
			const std::string& to,
			Eigen::Affine3d& t,
			ros::Time time = ros::Time(0));

	/**
	 * Loads a ROS plugin using pluginlib. It returns
	 * a shared pointer to an object of type Type. If
	 * an error occurs the program will be terminated
	 * immediately!
	 * IMPORTANT: Currently you will get warning when using this
	 * method, because of the destruction of the
	 * class loader!
	 * @package: the ROS package
	 * @library: the name of the library which has to be loaded
	 * @basePackage: the package of the base class
	 * @baseClass: full name of the base class (e.g. prm_planner::BaseClass)
	 */
	template<class Type>
	static boost::shared_ptr<Type> loadPlugin(const std::string& package,
			const std::string& library,
			const std::string& basePackage,
			const std::string& baseClass)
	{
		pluginlib::ClassLoader<Type> loader(basePackage, baseClass);

		boost::shared_ptr<Type> interface;

		try
		{
			interface = loader.createInstance(library);
		}
		catch (pluginlib::PluginlibException& ex)
		{
			LOG_FATAL("The plugin failed to load for some reason. Error: " << ex.what());
			exit(3);
		}

		return interface;
	}

	/**
	 * Automatically sets the transformation to the parent
	 * frame in each RGBDImage instance returned by the
	 * interface
	 */
	void setFrameNamesOfCamera(const std::string& cameraFrame,
			const std::string& cameraParentFrame);
	bool isMirrorImages() const;
	void setMirrorImages(bool mirrorImages);

protected:
	virtual void rosCallbackRGBD(const sensor_msgs::Image::ConstPtr& depthImage,
			const sensor_msgs::Image::ConstPtr& rgbImage);
	virtual void rosCallbackDepthOnly(const sensor_msgs::Image::ConstPtr& depthImage);
	virtual void rosCallbackRGBOnly(const sensor_msgs::Image::ConstPtr& rgbImage);
	virtual void rosCallbackDepthCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo);
	virtual void rosCallbackRGBCameraInfo(const sensor_msgs::CameraInfoPtr& camInfo);

	virtual void activateDepthAndRGBCamera(const std::string& topicRGBImage,
			const std::string& topicDepthImage);
	virtual void activateDepthCamera(const std::string& topicDepthImage);
	virtual void activateRGBCamera(const std::string& topicRGBImage);
	virtual void activateRGBCameraInfo(const std::string& topicRGBImageInfo);
	virtual void activateDepthCameraInfo(const std::string& topicDepthImageInfo);

protected:
	std::unordered_map<std::string, bool> activeCallbacks;

	bool depthAndRGBImageCallbackActive;
	bool depthCallbackActive;
	bool rgbCallbackActive;
	bool depthCameraInfoCallbackActive;
	bool rgbCameraInfoCallbackActive;

	ros::AsyncSpinner* spinner;

	ros::Subscriber subCameraDepth;
	ros::Subscriber subCameraRGB;
	ros::Subscriber subCameraInfoDepth;
	ros::Subscriber subCameraInfoRGB;

	message_filters::Subscriber<sensor_msgs::Image>* syncedDepthSub;
	message_filters::Subscriber<sensor_msgs::Image>* syncedColorSub;
	message_filters::Synchronizer<SyncPolicy>* syncronizer;

	sensor_msgs::CameraInfo::Ptr camInfoDepth;
	sensor_msgs::CameraInfo::Ptr camInfoRGB;
	bool camInfoDepthReceived, camInfoRGBReceived;

	std::string m_cameraParentFrameName;
	std::string m_cameraFrameName;

	bool m_mirrorImages;

	static tf::TransformListener* m_listener;
};

} /* namespace ais_ros */

#endif /* KUHNERD_LIBS_DK_UTIL_INCLUDE_DK_UTIL_ROS_BASE_INTERFACE_H_ */
