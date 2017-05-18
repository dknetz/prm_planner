/*
 * This file (serialization.h) is part of the "ais_ros"-package of Daniel Kuhner.
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
 * created:		Mar 4, 2014
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef SPECIAL_OBJECT_SERIALIZATION_H_
#define SPECIAL_OBJECT_SERIALIZATION_H_

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

#include <boost/config.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/collections_save_imp.hpp>
#include <boost/serialization/collections_load_imp.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include <opencv2/core/core.hpp>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>

namespace boost
{
namespace serialization
{

// ///////////////////////////////////////////////////////
//for cameraInfo
template<class Archive, class T>
inline void serialize(
		Archive & ar,
		sensor_msgs::CameraInfo_<T> & t,
		const unsigned int file_version
		)
{
	ar & t.D & t.K & t.P & t.R &
			t.binning_x & t.binning_y &
			t.distortion_model &
			t.header.frame_id & t.header.seq & t.header.stamp.nsec & t.header.stamp.sec &
			t.height &
			t.roi.do_rectify & t.roi.height & t.roi.width & t.roi.x_offset & t.roi.y_offset &
			t.width;
}

// ///////////////////////////////////////////////////////
//for ros::Time
template<class Archive>
inline void serialize(
		Archive & ar,
		ros::Time & t,
		const unsigned int file_version
		)
{
	ar & t.nsec & t.sec;
}
}
}

#endif /* SPECIAL_OBJECT_SERIALIZATION_H_ */
