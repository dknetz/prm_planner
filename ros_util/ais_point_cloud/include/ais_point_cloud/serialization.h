/*
 * serialization.h
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#ifndef ROBOTIC_LIBS_POINT_CLOUD_SERIALIZATION_H_
#define ROBOTIC_LIBS_POINT_CLOUD_SERIALIZATION_H_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/set.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace boost
{
namespace serialization
{

//pcl point cloud
template<class Archive, class PointType>
inline void load(
		Archive & ar,
		pcl::PointCloud<PointType> & t,
		const unsigned int file_version
		)
{
	ar & t.header.frame_id;
	ar & t.header.seq;
	ar & t.header.stamp;
	ar & t.height;
	ar & t.is_dense;
	ar & t.points;
	ar & t.sensor_orientation_;
	ar & t.sensor_origin_;
	ar & t.width;

//we set the time directly, otherwise they cannot be visualized in rviz
	t.header.stamp = ros::Time::now().toNSec() / 1e3;
}

//pcl point cloud
template<class Archive, class PointType>
inline void save(
		Archive & ar,
		const pcl::PointCloud<PointType> & t,
		const unsigned int file_version
		)
{
	ar & t.header.frame_id;
	ar & t.header.seq;
	ar & t.header.stamp;
	ar & t.height;
	ar & t.is_dense;
	ar & t.points;
	ar & t.sensor_orientation_;
	ar & t.sensor_origin_;
	ar & t.width;
}

//pcl point cloud
template<class Archive, class PointType>
inline void serialize(
		Archive & ar,
		pcl::PointCloud<PointType> & t,
		const unsigned int file_version
		)
{
	split_free(ar, t, file_version);
}

template<class Archive>
inline void load(
		Archive & ar,
		pcl::PointXYZRGB & t,
		const unsigned int file_version
		)
{
	ar & t.r;
	ar & t.g;
	ar & t.b;
	ar & t.a;

	bool isNan;
	ar & isNan;

	if (isNan)
	{
		t.x = std::numeric_limits<float>::quiet_NaN();
		t.y = std::numeric_limits<float>::quiet_NaN();
		t.z = std::numeric_limits<float>::quiet_NaN();

		float dummy;
		ar & dummy; //x
		ar & dummy; //y
		ar & dummy; //z
	}
	else
	{
		ar & t.x;
		ar & t.y;
		ar & t.z;
	}
}

template<class Archive>
inline void save(
		Archive & ar,
		const pcl::PointXYZRGB & t,
		const unsigned int file_version
		)
{
//color
	if (isnan(t.r) || isnan(t.g) || isnan(t.b) || isnan(t.a))
	{
		uint8_t dummy = 0;
		ar & dummy & dummy & dummy & dummy;
	}
	else
	{
		ar & t.r & t.g & t.b & t.a; //r, g, b, a
	}

//position
	bool isNan = (isnan(t.x) || isnan(t.y) || isnan(t.z));
	ar & isNan; //is nan

	float x = isnan(t.x) ? 1e10 : t.x;
	float y = isnan(t.y) ? 1e10 : t.y;
	float z = isnan(t.z) ? 1e10 : t.z;
	ar & x;
	ar & y;
	ar & z;
}

// ///////////////////////////////////////////////////////
template<class Archive>
inline void load(
		Archive & ar,
		pcl::PointNormal & t,
		const unsigned int file_version
		)
{
	ar & t.curvature;

	ar & t.normal_x;
	ar & t.normal_y;
	ar & t.normal_z;

	bool isNan;
	ar & isNan;

	if (isNan)
	{
		t.x = std::numeric_limits<float>::quiet_NaN();
		t.y = std::numeric_limits<float>::quiet_NaN();
		t.z = std::numeric_limits<float>::quiet_NaN();

		float dummy;
		ar & dummy; //x
		ar & dummy; //y
		ar & dummy; //z
	}
	else
	{
		ar & t.x;
		ar & t.y;
		ar & t.z;
	}
}

template<class Archive>
inline void save(
		Archive & ar,
		const pcl::PointNormal & t,
		const unsigned int file_version
		)
{
//curvature
	float dummyf = 0;
	ar & (isnan(t.curvature) ? dummyf : t.curvature);

	if (isnan(t.normal_x) || isnan(t.normal_y) || isnan(t.normal_z))
	{
		ar & dummyf & dummyf & dummyf;
	}

//position
	ar & (isnan(t.x) || isnan(t.y) || isnan(t.z)); //is nan

	dummyf = 1e10;
	ar & (isnan(t.x) ? dummyf : t.x);
	ar & (isnan(t.y) ? dummyf : t.y);
	ar & (isnan(t.z) ? dummyf : t.z);
}

// ///////////////////////////////////////////////////////
template<class Archive>
inline void load(
		Archive & ar,
		pcl::PointXYZRGBNormal & t,
		const unsigned int file_version
		)
{
//color
	ar & t.r;
	ar & t.g;
	ar & t.b;
	ar & t.a;

//curvature
	ar & t.curvature;

//normals
	ar & t.normal_x;
	ar & t.normal_y;
	ar & t.normal_z;

//position
	bool isNan;
	ar & isNan;

	if (isNan)
	{
		t.x = std::numeric_limits<float>::quiet_NaN();
		t.y = std::numeric_limits<float>::quiet_NaN();
		t.z = std::numeric_limits<float>::quiet_NaN();

		float dummy;
		ar & dummy; //x
		ar & dummy; //y
		ar & dummy; //z
	}
	else
	{
		ar & t.x;
		ar & t.y;
		ar & t.z;
	}
}

template<class Archive>
inline void save(
		Archive & ar,
		const pcl::PointXYZRGBNormal & t,
		const unsigned int file_version
		)
{
	static const float dummyf = 0;
	static const float dummy2f = 1e10;
	static const uint8_t dummyu = 0;

//color
	if (isnan(t.r) || isnan(t.g) || isnan(t.b) || isnan(t.a))
	{
		ar & dummyu & dummyu & dummyu & dummyu;
	}
	else
	{
		ar & t.r & t.g & t.b & t.a; //r, g, b, a
	}

//curvature
	float curvature = isnan(t.curvature) ? dummyf : t.curvature;
	ar & curvature;

//normals
	if (isnan(t.normal_x) || isnan(t.normal_y) || isnan(t.normal_z))
	{
		ar & dummyf & dummyf & dummyf;
	}
	else
	{
		ar & t.normal_x & t.normal_y & t.normal_z;
	}

//position
	bool isNan = (isnan(t.x) || isnan(t.y) || isnan(t.z));
	ar & isNan; //is nan

	ar & (isnan(t.x) ? dummy2f : t.x);
	ar & (isnan(t.y) ? dummy2f : t.y);
	ar & (isnan(t.z) ? dummy2f : t.z);
}

// ///////////////////////////////////////////////////////
template<class Archive>
inline void load(
		Archive & ar,
		pcl::PointXYZ & t,
		const unsigned int file_version
		)
{
//position
	bool isNan;
	ar & isNan;

	if (isNan)
	{
		t.x = std::numeric_limits<float>::quiet_NaN();
		t.y = std::numeric_limits<float>::quiet_NaN();
		t.z = std::numeric_limits<float>::quiet_NaN();

		float dummy;
		ar & dummy; //x
		ar & dummy; //y
		ar & dummy; //z
	}
	else
	{
		ar & t.x;
		ar & t.y;
		ar & t.z;
	}
}

template<class Archive>
inline void save(
		Archive & ar,
		const pcl::PointXYZ & t,
		const unsigned int file_version
		)
{
//position
	ar & (isnan(t.x) || isnan(t.y) || isnan(t.z)); //is nan

	float dummyf = 1e10;
	ar & (isnan(t.x) ? dummyf : t.x);
	ar & (isnan(t.y) ? dummyf : t.y);
	ar & (isnan(t.z) ? dummyf : t.z);
}

// ///////////////////////////////////////////////////////
//	OpenCV
/** Serialization support for cv::Mat */
template<class Archive>
void save(Archive & ar,
		const ::cv::Mat& m,
		const unsigned int version)
{
	size_t elem_size = m.elemSize();
	size_t elem_type = m.type();

	ar & m.cols;
	ar & m.rows;
	ar & elem_size;
	ar & elem_type;

	const size_t data_size = m.cols * m.rows * elem_size;
	ar & boost::serialization::make_array(m.ptr(), data_size);
}

/** Serialization support for cv::Mat */
template<class Archive>
void load(Archive & ar,
		::cv::Mat& m,
		const unsigned int version)
{
	int cols, rows;
	size_t elem_size, elem_type;

	ar & cols;
	ar & rows;
	ar & elem_size;
	ar & elem_type;

	m.create(rows, cols, elem_type);

	size_t data_size = m.cols * m.rows * elem_size;
	ar & boost::serialization::make_array(m.ptr(), data_size);
}

// ------------- cv::Point_
template<class Archive, class T>
void serialize(Archive & ar,
		cv::Point_<T>& point,
		const unsigned int version)
{
	ar & point.x;
	ar & point.y;
}

// ------------- cv::Point3_
template<class Archive, class T>
void serialize(Archive & ar,
		cv::Point3_<T>& point,
		const unsigned int version)
{
	ar & point.x;
	ar & point.y;
	ar & point.z;
}

// ------------- cv::Size_

template<class Archive, class T>
void serialize(Archive & ar,
		cv::Size_<T>& size,
		const unsigned int version)
{
	ar & size.width;
	ar & size.height;
}

// ------------- cv::Rect_
template<class Archive, class T>
void serialize(Archive & ar,
		cv::Rect_<T>& rect,
		const unsigned int version)
{
	ar & rect.x;
	ar & rect.y;
	ar & rect.width;
	ar & rect.height;
}

// ------------- cv::RotatedRect
template<class Archive>
void serialize(Archive & ar,
		cv::RotatedRect& rrect,
		const unsigned int version)
{
	ar & rrect.center;
	ar & rrect.size;
	ar & rrect.angle;
}

// ------------- cv::Vec
template<class Archive, class T, int n>
void serialize(Archive & ar,
		cv::Vec<T, n>& vec,
		const unsigned int version)
{
	ar & vec.val;
}

// ------------- cv::Scalar_
template<class Archive, class T>
void serialize(Archive & ar,
		cv::Scalar_<T>& scalar,
		const unsigned int version)
{
	ar & scalar.val;
}

// ------------- cv::Range
template<class Archive>
void serialize(Archive & ar,
		cv::Range& range,
		const unsigned int version)
{
	ar & range.start;
	ar & range.end;
}

} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat);
BOOST_SERIALIZATION_SPLIT_FREE(pcl::PointXYZ);
BOOST_SERIALIZATION_SPLIT_FREE(pcl::PointNormal);
BOOST_SERIALIZATION_SPLIT_FREE(pcl::PointXYZRGBNormal);
BOOST_SERIALIZATION_SPLIT_FREE(pcl::PointXYZRGB);

#endif /* ROBOTIC_LIBS_POINT_CLOUD_SERIALIZATION_H_ */
