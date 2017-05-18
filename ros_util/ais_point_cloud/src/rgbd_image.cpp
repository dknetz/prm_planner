/*
 * Copyright (c) 2013 Daniel Kuhner.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Daniel Kuhner - kuhnerd@informatik.uni-freiburg.de
 */

#include <ais_definitions/exception.h>
#include <ais_point_cloud/rgbd_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ais_log/log.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/time.h>
#include <limits>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv/highgui.h>
#else
#include <opencv2/highgui.hpp>
#endif

using namespace ais_util;

namespace ais_point_cloud
{

RGBDImage::RGBDImage(cv::Mat& depth) :
				m_height(depth.rows),
				m_width(depth.cols),
				m_depthImage(depth),
				m_cloud(new MyPointCloud()),
				m_cloudComputed(false),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(false)
{
}

RGBDImage::RGBDImage(cv::Mat& depth,
		const RGBDImage::CameraInfo& depthCameraInfo) :
				m_height(depth.rows),
				m_width(depth.cols),
				m_depthImage(depth),
				m_cloud(new MyPointCloud()),
				m_cloudComputed(false),
				m_depthCameraInfo(depthCameraInfo),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(false)
{
}

RGBDImage::RGBDImage(cv::Mat& color,
		cv::Mat& depth,
		MyPointCloudP& cloud) :
				m_height(color.rows),
				m_width(color.cols),
				m_rgbImage(color),
				m_depthImage(depth),
				m_cloud(cloud),
				m_cloudComputed(false),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(true)
{
	assert(m_rgbImage.rows == m_depthImage.rows && m_rgbImage.cols == m_depthImage.cols);
}

RGBDImage::RGBDImage(cv::Mat& color,
		cv::Mat& depth) :
				m_height(color.rows),
				m_width(color.cols),
				m_rgbImage(color),
				m_depthImage(depth),
				m_cloud(new MyPointCloud()),
				m_cloudComputed(false),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(true)
{
	assert(m_rgbImage.rows == m_depthImage.rows && m_rgbImage.cols == m_depthImage.cols);
}

RGBDImage::RGBDImage(cv::Mat& color,
		cv::Mat& depth,
		const RGBDImage::CameraInfo& depthCameraInfo) :
				m_height(color.rows),
				m_width(color.cols),
				m_rgbImage(color),
				m_depthImage(depth),
				m_cloud(new MyPointCloud()),
				m_cloudComputed(false),
				m_depthCameraInfo(depthCameraInfo),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(true)
{
	assert(m_rgbImage.rows == m_depthImage.rows && m_rgbImage.cols == m_depthImage.cols);
}

RGBDImage::RGBDImage(const RGBDImage &image) :
				m_height(image.getHeight()),
				m_width(image.getWidth()),
				m_rgbImage(image.getRGB()),
				m_depthImage(image.getDepth()),
				m_cloud(image.m_cloud),
				m_time(image.m_time),
				m_cloudComputed(image.m_cloudComputed),
				m_depthCameraInfo(image.m_depthCameraInfo),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(image.m_hasRGB)
{
}

RGBDImage::RGBDImage(const RGBDImage::Ptr& image) :
				m_height(image->getHeight()),
				m_width(image->getWidth()),
				m_rgbImage(image->getRGB()),
				m_depthImage(image->getDepth()),
				m_cloud(image->m_cloud),
				m_time(image->m_time),
				m_cloudComputed(image->m_cloudComputed),
				m_depthCameraInfo(image->m_depthCameraInfo),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(image->m_hasRGB)
{
}

RGBDImage::RGBDImage() :
				m_height(0),
				m_width(0),
				m_cloud(new MyPointCloud()),
				m_cloudComputed(false),
				m_showHighGuiDepthImage(false),
				m_showHighGuiRGBImage(false),
				m_doStatisticalOutlierRemoval(false),
				m_mirrorImages(false),
				m_hasRGB(false)
{
}

RGBDImage::~RGBDImage()
{
}

RGBDImage::CameraInfo::CameraInfo(const double& cx,
		const double& cy,
		const double& fx,
		const double& fy,
		const double& width,
		const double& height) :
				cx(cx),
				cy(cy),
				fx(fx),
				fy(fy),
				width(width),
				height(height)
{
}

RGBDImage::CameraInfo::CameraInfo() :
				cx(0),
				cy(0),
				fx(0),
				fy(0),
				width(0),
				height(0)
{
}

RGBDImage::OutlierRemovalParams::OutlierRemovalParams() :
				meanK(10),
				stdDevThreshold(1.0)
{
}

void RGBDImage::convertColorToWorld(const sensor_msgs::CameraInfo::Ptr& camInfo,
		int x,
		int y,
		double depth,
		double& xWorld,
		double& yWorld,
		double& zWorld)
{
	Eigen::Vector2d p(x, y);
	Eigen::Vector3d result = convertColorToWorld(camInfo, p, depth);
	xWorld = result.x();
	yWorld = result.y();
	zWorld = result.z();
}

Eigen::Vector3d RGBDImage::convertColorToWorld(const sensor_msgs::CameraInfo::Ptr& camInfo,
		const Eigen::Vector2d& color,
		double depth)
{
	static const double fx_rgb = camInfo->K.elems[0]; //525.0
	static const double fy_rgb = camInfo->K.elems[4]; //525
	static const double cx_rgb = camInfo->K.elems[2]; //319.5
	static const double cy_rgb = camInfo->K.elems[5]; //239.5

	Eigen::Vector3d result((color.x() - cx_rgb) * depth / fx_rgb, (color.y() - cy_rgb) * depth / fy_rgb, depth);

	return result;
}

RGBDImage::RGBD RGBDImage::operator ()(const int y,
		const int x) const
		{
	double depth = m_depthImage.at<float>(y, x);

	if (m_hasRGB)
	{
		cv::Vec3b rgb = m_rgbImage.at<cv::Vec3b>(y, x);
		return RGBD(rgb[2], rgb[1], rgb[0], depth);
	}
	else
	{
		return RGBD(depth);
	}

}

//void RGBDImage::operator =(const RGBDImage &image)
//{
//	m_height = image.getHeight();
//	m_width = image.getWidth();
//	m_rgbImage = image.getRGB();
//	m_depthImage = image.getDepth();
//	m_cloud = image.m_cloud;
//}

cv::Vec3b& RGBDImage::atC(const int x,
		const int y)
{
	if (m_hasRGB)
		return m_rgbImage.at<cv::Vec3b>(y, x);
	else
		throw ais_definitions::Exception("No RGB data available");
}

float& RGBDImage::atD(const int x,
		const int y)
{
	return m_depthImage.at<float>(y, x);
}

void RGBDImage::computeCloud()
{
	if (!m_cloudComputed)
	{
		if (m_depthCameraInfo.cx == 0)
		{
			LOG_WARNING("The camera depth info was maybe not set so far!");
		}

		m_cloud->resize(m_depthImage.cols * m_depthImage.rows);
		m_cloud->is_dense = false;
		m_cloud->height = m_depthImage.rows;
		m_cloud->width = m_depthImage.cols;

		m_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
		m_cloud->header.frame_id = m_frame;

		float* depthImageData = (float*) (m_depthImage.data);
		cv::Vec3b* colorImageData = (cv::Vec3b*) (m_rgbImage.data);

//#		pragma omp parallel for
		for (int y = 0; y < m_depthImage.rows; ++y)
		{
			Eigen::Vector3d pos3d;
			double depth;
			for (int x = 0; x < m_depthImage.cols; ++x)
			{
				depth = (double) depthImageData[m_depthImage.cols * y + x];

				//ignore values > 7.0 m (robot self filter), TODO: make it a parameter
				if (depth > 7.0)
				{
					pos3d = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
							std::numeric_limits<double>::quiet_NaN());
				}
				else
				{
//					LOG_INFO(m_depthCameraInfo.cx << " " << m_depthCameraInfo.cy << " "  <<m_depthCameraInfo.fx << " " << m_depthCameraInfo.fy);
					convert2DTo3DDepth((double) x, (double) y, depth, pos3d.x(), pos3d.y(),
							pos3d.z());
					pos3d = m_transformation * pos3d;
				}

				MyPointType& p = m_cloud->points[y * m_depthImage.cols + x];

				p.x = pos3d.x();
				p.y = pos3d.y();
				p.z = pos3d.z();

				if (m_hasRGB)
				{
					cv::Vec3b& col = colorImageData[m_rgbImage.cols * y + x];
					p.r = col[0];
					p.g = col[1];
					p.b = col[2];
				}
			}
		}

		if (m_doStatisticalOutlierRemoval)
		{
			MyPointCloudP cloudFiltered(new MyPointCloud);
			pcl::StatisticalOutlierRemoval<MyPointType> sor;
			sor.setInputCloud(m_cloud);
			sor.setMeanK(m_outlierRemovalParams.meanK);
			sor.setStddevMulThresh(m_outlierRemovalParams.stdDevThreshold);
			sor.setKeepOrganized(true);
			sor.setUserFilterValue(std::numeric_limits<double>::quiet_NaN());
			sor.filter(*cloudFiltered);
			m_cloud = cloudFiltered;
		}

		m_cloudComputed = true;
	}
}

MyPointCloudP RGBDImage::computeNanFilteredDownsampledCloud(const int level,
		const bool cache)
{
	MyPointCloudP cloud = getCloud();
	MyPointCloudP cloudDownsampled(new MyPointCloud);
	cloudDownsampled->header = cloud->header;
	cloudDownsampled->is_dense = true;
	cloudDownsampled->resize((m_depthImage.rows / level + 1) * (m_depthImage.cols / level + 1));

	int j = 0;
	for (int y = 0; y < m_depthImage.rows; y += level)
	{
		for (int x = 0; x < m_depthImage.cols; x += level)
		{
			auto p = cloud->points[y * m_depthImage.cols + x];

			//filter nan/inf
			if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
			{
				continue;
			}

			cloudDownsampled->points[j++] = p;
		}
	}

	cloudDownsampled->resize(j);
	cloudDownsampled->height = 1;
	cloudDownsampled->width = cloudDownsampled->size();

	if (cache)
	{
		m_downsampledCloud[level] = cloudDownsampled;
	}

	return cloudDownsampled;
}

bool RGBDImage::isOnImage(const double& x,
		const double& y)
{
	return x > 0 && x < m_width && y > 0 && y < m_height;
}

void RGBDImage::project3D(const Eigen::Vector3d& in,
		Eigen::Vector3d& out)
{
	static const double cx = m_depthCameraInfo.cx;
	static const double cy = m_depthCameraInfo.cy;
	static const double fx = m_depthCameraInfo.fx;
	static const double fy = m_depthCameraInfo.fy;
	out.x() = (int) (in.x() / (in.z() * fx) + cx);
	out.y() = (int) (in.y() / (in.z() * fy) + cy);
	out.z() = in.z();

}

const MyPointCloudP& RGBDImage::getNanFilteredCloud()
{
	if (m_filteredCloud.get() == NULL)
	{
		MyPointCloudP cloud = getCloud();
		m_filteredCloud.reset(new MyPointCloud);
		pcl::removeNaNFromPointCloud(*cloud, *m_filteredCloud, m_filterIndexMapping);
	}

	return m_filteredCloud;
}

const std::vector<int>& RGBDImage::getFilterIndexMapping()
{
	if (m_filteredCloud.get() == NULL)
	{
		getNanFilteredCloud();
	}

	return m_filterIndexMapping;
}

void RGBDImage::showDepthImage()
{
	if (!m_showHighGuiDepthImage)
	{
		cv::namedWindow("depth_image");
		m_showHighGuiDepthImage = true;
	}

	cv::Mat depthNormalized;
	cv::normalize(m_depthImage, depthNormalized, 255);
	cv::imshow("depth_image", depthNormalized);
	cv::waitKey(10);
}

void RGBDImage::showRGBImage()
{
	if (m_hasRGB)
	{
		if (!m_showHighGuiDepthImage)
		{
			cv::namedWindow("rgb_image");
			m_showHighGuiDepthImage = true;
		}

		cv::imshow("rgb_image", m_rgbImage);
		cv::waitKey(0);
	}
	else
		throw ais_definitions::Exception("No RGB data available");
}

bool RGBDImage::isDoStatisticalOutlierRemoval() const
{
	return m_doStatisticalOutlierRemoval;
}

RGBDImage::OutlierRemovalParams RGBDImage::getOutlierRemovalParams() const
{
	return m_outlierRemovalParams;
}

void RGBDImage::setOutlierRemovalParams(OutlierRemovalParams outlierRemovalParams)
{
	m_outlierRemovalParams = outlierRemovalParams;
}

void RGBDImage::setDoStatisticalOutlierRemoval(bool doStatisticalOutlierRemoval)
{
	m_doStatisticalOutlierRemoval = doStatisticalOutlierRemoval;
}

bool RGBDImage::isMirrorImages() const
{
	return m_mirrorImages;
}

void RGBDImage::setMirrorImages(bool mirrorImages)
{
	m_mirrorImages = mirrorImages;
}

cv::Mat RGBDImage::getRGB() const
{
	if (!m_hasRGB)
	{
		throw ais_definitions::Exception("No RGB data available");
	}
	return m_rgbImage;
}

} /* namespace ais_point_cloud */

