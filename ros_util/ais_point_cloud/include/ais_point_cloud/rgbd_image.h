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

#ifndef ROBOTIC_LIBS_POINT_CLOUD_RGBD_IMAGE_H_
#define ROBOTIC_LIBS_POINT_CLOUD_RGBD_IMAGE_H_

#include <ais_definitions/macros.h>
#include <ais_util/time.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <ais_point_cloud/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <unordered_map>

namespace ais_point_cloud
{

class RGBDImage
{
public:
	typedef boost::shared_ptr<RGBDImage> Ptr;

	struct RGBD
	{
		bool hasColor;
		uchar r, g, b;
		double d;
		RGBD(uchar r,
				uchar g,
				uchar b,
				double d) :
						r(r),
						g(g),
						b(b),
						d(d),
						hasColor(true)
		{
		}
		RGBD(double d) :
						r(0),
						g(0),
						b(0),
						d(d),
						hasColor(false)
		{
		}
	};

	struct CameraInfo
	{
		CameraInfo(const double& cx,
				const double& cy,
				const double& fx,
				const double& fy,
				const double& width,
				const double& height);
		CameraInfo();
		const double cx;
		const double cy;
		const double fx;
		const double fy;
		const double width;
		const double height;
	};

	struct OutlierRemovalParams
	{
		OutlierRemovalParams();
		double meanK;
		double stdDevThreshold;
	};

public:
	//depth only
	RGBDImage(cv::Mat& depth);
	RGBDImage(cv::Mat& depth,
			const RGBDImage::CameraInfo& depthCameraInfo);

	//rgbd
	RGBDImage(cv::Mat& color,
			cv::Mat& depth,
			MyPointCloudP& cloud);
	RGBDImage(cv::Mat& color,
			cv::Mat& depth,
			const RGBDImage::CameraInfo& depthCameraInfo);
	RGBDImage(cv::Mat& color,
			cv::Mat& depth);
	RGBDImage();
	RGBDImage(RGBDImage const &image);
	RGBDImage(RGBDImage::Ptr const &image);

	virtual ~RGBDImage();

	inline bool isValidIndex(int x,
			int y) const
			{
		return x >= 0 && x < m_width && y >= 0 && y < m_height;
	}

	inline bool hasData(int x,
			int y) const
			{
		return (m_depthImage.at<float>(y, x) == m_depthImage.at<float>(y, x)) && m_depthImage.at<float>(y, x) >= 0.1
				&& m_depthImage.at<float>(y, x) < 100.0;
	}

	inline int getHeight() const
	{
		return m_height;
	}

	inline int getWidth() const
	{
		return m_width;
	}

	static void convertColorToWorld(const sensor_msgs::CameraInfo::Ptr& camInfo,
			int x,
			int y,
			double depth,
			double& xWorld,
			double& yWorld,
			double& zWorld);

	static Eigen::Vector3d convertColorToWorld(const sensor_msgs::CameraInfo::Ptr& camInfo,
			const Eigen::Vector2d& color,
			double depth);

	RGBD operator()(int const y,
			int const x) const;
	//	void operator=(RGBDImage const &image);

	cv::Vec3b& atC(int const x,
			int const y);

	float& atD(int const x,
			int const y);

	cv::Mat getRGB() const;

	inline cv::Mat getDepth() const
	{
		return m_depthImage;
	}

	inline MyPointCloudP getCloud()
	{
		if (!m_cloudComputed)
		{
			computeCloud();
		}

		return m_cloud;
	}

	inline void setTransformation(const Eigen::Affine3d& transformation)
	{
		m_transformation = transformation;
	}

	inline Eigen::Affine3d getTransformation()
	{
		return m_transformation;
	}

	inline void setFrame(const std::string& frame)
	{
		m_frame = frame;
	}

	inline std::string getFrame()
	{
		return m_frame;
	}

	inline Eigen::Vector3d getTransformedPoint(const Eigen::Vector3d& point)
	{
		return m_transformation * point;
	}

	const MyPointCloudP& getNanFilteredCloud();
	const std::vector<int>& getFilterIndexMapping();

	/**
	 * return the downsampled version of the cloud
	 * based on the level. If you choose a level of
	 * 2, than the cloud gets downsampled by to 2.
	 * Futhermore, the point cloud gets cleaned from
	 * nan's
	 */
	inline MyPointCloudP getNanFilteredDownsampledCloud(const int level,
			bool cache = true)
	{
		if (!cache)
		{
			return computeNanFilteredDownsampledCloud(level, cache);
		}
		else
		{
			if (!CHECK_MAP(m_downsampledCloud, level))
			{
				return computeNanFilteredDownsampledCloud(level, cache);
			}
			else
			{
				return m_downsampledCloud[level];
			}
		}
	}

	void project3D(const Eigen::Vector3d& in,
			Eigen::Vector3d& out);
	bool isOnImage(const double& x,
			const double& y);

	void computeCloud();
	MyPointCloudP computeNanFilteredDownsampledCloud(const int level,
			const bool cache);

	void showRGBImage();
	void showDepthImage();
	bool isDoStatisticalOutlierRemoval() const;
	void setDoStatisticalOutlierRemoval(bool doStatisticalOutlierRemoval);
	OutlierRemovalParams getOutlierRemovalParams() const;
	void setOutlierRemovalParams(OutlierRemovalParams outlierRemovalParams);
	bool isMirrorImages() const;
	void setMirrorImages(bool mirrorImages);

	int m_height;
	int m_width;
	cv::Mat m_rgbImage;
	cv::Mat m_depthImage;
	MyPointCloudP m_cloud;
	MyPointCloudP m_filteredCloud;
	std::vector<int> m_filterIndexMapping;
	std::unordered_map<int, MyPointCloudP> m_downsampledCloud;
	ais_util::Time m_time;
	bool m_cloudComputed;
	CameraInfo m_depthCameraInfo;
	bool m_showHighGuiDepthImage;
	bool m_showHighGuiRGBImage;
	bool m_doStatisticalOutlierRemoval;
	OutlierRemovalParams m_outlierRemovalParams;
	Eigen::Affine3d m_transformation;
	std::string m_frame;
	bool m_mirrorImages;
	bool m_hasRGB;

private:
	inline void convert2DTo3DDepth(const double& x,
			const double& y,
			const double& depth,
			double& xOut,
			double& yOut,
			double& zOut)
	{
		static const double cx = m_depthCameraInfo.cx;
		static const double cy = m_depthCameraInfo.cy;
		static const double fx = m_depthCameraInfo.fx;
		static const double fy = m_depthCameraInfo.fy;
		if (m_mirrorImages)
		{
			xOut = (((m_depthCameraInfo.width - x - 1) - cx) * depth * fx);
			yOut = (((m_depthCameraInfo.height - y - 1) - cy) * depth * fy);
		}
		else
		{
			xOut = ((x - cx) * depth * fx);
			yOut = ((y - cy) * depth * fy);
		}
		zOut = depth;
	}
};

} /* namespace ais_point_cloud */
#endif /* ROBOTIC_LIBS_POINT_CLOUD_RGBD_IMAGE_H_ */
