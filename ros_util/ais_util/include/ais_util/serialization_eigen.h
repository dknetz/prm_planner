/*
 * This file (serialization.h) is part of the "ais_definitions" packages of Daniel Kuhner.
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
#ifndef _9ef0857dc95505dc28dc9714e6109c3f
#define _9ef0857dc95505dc28dc9714e6109c3f

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/collections_save_imp.hpp>
#include <boost/serialization/collections_load_imp.hpp>
#include <boost/serialization/split_free.hpp>

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <Eigen/Core>

#include <ais_log/log.h>

namespace boost
{
namespace serialization
{
//for Eigen::Matrix
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save(Archive & ar,
		const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	int rows = m.rows(), cols = m.cols();
	ar & rows;
	ar & cols;
	for (size_t i = 0; i < rows * cols; ++i)
	{
		bool nan = std::isnan(m.data()[i]);
		_Scalar value = nan ? 0 : m.data()[i];
		ar & nan & value;
	}
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void load(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	int rows, cols;
	ar & rows;
	ar & cols;
	m.resize(rows, cols);
	for (size_t i = 0; i < rows * cols; ++i)
	{
		bool nan;
		_Scalar value;
		ar & nan & value;

		if (nan)
		{
			m.data()[i] = std::numeric_limits<_Scalar>::quiet_NaN();
		}
		else
		{
			m.data()[i] = value;
		}
	}
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void serialize(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m,
		const unsigned int version)
{
	split_free(ar, m, version);
}

//for Eigen::Transform
template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void serialize(
		Archive & ar,
		Eigen::Transform<_Scalar, _Dim, _Mode, _Options> & t,
		const unsigned int file_version
		)
{
	ar & t.matrix();
}

//for Eigen::Quaternion
template<class Archive, typename _Scalar>
inline void serialize(
		Archive & ar,
		Eigen::Quaternion<_Scalar> & t,
		const unsigned int file_version
		)
{
	ar & t.x();
	ar & t.y();
	ar & t.z();
	ar & t.w();
}

} // namespace serialization
} // namespace boost

#endif /* _9ef0857dc95505dc28dc9714e6109c3f */
