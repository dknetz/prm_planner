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
#ifndef _c201d3cc58b1c333379b3fe2040cab25
#define _c201d3cc58b1c333379b3fe2040cab25

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

#include <boost/config.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <iosfwd>

#include <kdl/jntarray.hpp>

#include <ais_log/log.h>

#define SERIALIZE_TO_FILE(FILENAME, VARIABLES)\
		{\
			std::ofstream ofs(FILENAME, std::fstream::binary);\
			boost::archive::binary_oarchive oa(ofs);\
			oa << VARIABLES;\
		}

#define UNSERIALIZE_FROM_FILE(PATH, VARIABLES)\
		if (boost::filesystem::exists(PATH))\
		{\
			std::ifstream ifs(PATH, std::fstream::binary);\
			boost::archive::binary_iarchive ia(ifs);\
			ia >> VARIABLES;\
		}\
		else\
		{\
			LOG_ERROR("The file PATH doesn't exists.");\
		}

#define SERIALIZATION_METHOD_EMPTY()\
		private:\
			friend class boost::serialization::access;\
			template<class Archive>\
			void serialize(Archive & ar,\
					const unsigned int version)\
			{\
			}\

#define SERIALIZATION_METHOD(VARIABLES)\
		private:\
			friend class boost::serialization::access;\
			template<class Archive>\
			void serialize(Archive & ar,\
					const unsigned int version)\
			{\
				ar & VARIABLES;\
			}\

#define SERIALIZATION_METHOD_INIT() private:\
			friend class boost::serialization::access;

#define SERIALIZATION_METHOD_LOAD(VARIABLES, OTHER_STUFF_TO_DO)\
		private:\
			template<class Archive>\
			void load(Archive & ar,\
					const unsigned int version)\
			{\
				ar & VARIABLES;\
				OTHER_STUFF_TO_DO;\
			}\

#define SERIALIZATION_METHOD_SAVE(VARIABLES, OTHER_STUFF_TO_DO)\
		private:\
			template<class Archive>\
			void save(Archive & ar,\
					const unsigned int version) const\
			{\
				ar & VARIABLES;\
				OTHER_STUFF_TO_DO;\
			}\

#define SERIALIZATION_METHOD_SPLITTED(VARIABLES, OTHER_STUFF_TO_DO_LOAD, OTHER_STUFF_TO_DO_SAVE)\
	SERIALIZATION_METHOD_INIT()\
	SERIALIZATION_METHOD_LOAD(VARIABLES, OTHER_STUFF_TO_DO_LOAD)\
	SERIALIZATION_METHOD_SAVE(VARIABLES, OTHER_STUFF_TO_DO_SAVE)\
	BOOST_SERIALIZATION_SPLIT_MEMBER()

#define SERIALIZATION_METHOD_SPLITTED_LOAD(VARIABLES, OTHER_STUFF_TO_DO_LOAD)\
	SERIALIZATION_METHOD_INIT()\
	SERIALIZATION_METHOD_LOAD(VARIABLES, OTHER_STUFF_TO_DO_LOAD)\
	SERIALIZATION_METHOD_SAVE(VARIABLES,)\
	BOOST_SERIALIZATION_SPLIT_MEMBER()

#define SERIALIZATION_METHOD_SPLITTED_SAVE(VARIABLES, OTHER_STUFF_TO_DO_SAVE)\
	SERIALIZATION_METHOD_INIT()\
	SERIALIZATION_METHOD_LOAD(VARIABLES,)\
	SERIALIZATION_METHOD_SAVE(VARIABLES, OTHER_STUFF_TO_DO_SAVE)\
	BOOST_SERIALIZATION_SPLIT_MEMBER()

#define SERIALIZATION_WITH_PARENT_METHOD(PARENT, VARIABLES)\
		private:\
			friend class boost::serialization::access;\
			template<class Archive>\
			void serialize(Archive & ar,\
					const unsigned int version)\
			{\
				ar & boost::serialization::base_object<PARENT>(*this);\
				ar & VARIABLES;\
			}\

#define SERIALIZATION_REG_HEADER(CLASSNAME)\
		BOOST_CLASS_EXPORT_KEY(CLASSNAME)

#define SERIALIZATION_REG_SOURCE(CLASSNAME)\
		BOOST_CLASS_EXPORT_IMPLEMENT(CLASSNAME)

#define SERIALIZE_PARENT(TYPE) boost::serialization::base_object<TYPE>(*this)

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
		bool nan = isnan(m.data()[i]);
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

//for KDL::JntArray
template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void serialize(
		Archive & ar,
		KDL::JntArray & t,
		const unsigned int file_version
		)
{
	ar & t.data;
}

// ///////////////////////////////////////////////////////
//std unordered map
template<class Archive, class Type, class Key, class Hash, class Pred, class Allocator>
inline void save(
		Archive & ar,
		const std::unordered_map<Key, Type, Hash, Pred, Allocator> &t,
		const unsigned int /* file_version */
		)
{
	boost::serialization::stl::save_collection<
			Archive,
			std::unordered_map<Key, Type, Hash, Pred, Allocator>
	>(ar, t);
}

template<class Archive, class Type, class Key, class Hash, class Pred, class Allocator>
inline void load(
		Archive & ar,
		std::unordered_map<Key, Type, Hash, Pred, Allocator> &t,
		const unsigned int /* file_version */
		)
{
	boost::serialization::stl::load_collection<
			Archive,
			std::unordered_map<Key, Type, Hash, Pred, Allocator>,
			boost::serialization::stl::archive_input_map<
					Archive, std::unordered_map<Key, Type, Hash, Pred, Allocator> >,
			boost::serialization::stl::no_reserve_imp<std::unordered_map<
					Key, Type, Hash, Pred, Allocator
					>
			>
	>(ar, t);
}

//// split non-intrusive serialization function member into separate
//// non intrusive save/load member functions
template<class Archive, class Type, class Key, class Hash, class Pred, class Allocator>
inline void serialize(
		Archive & ar,
		std::unordered_map<Key, Type, Hash, Pred, Allocator> &t,
		const unsigned int file_version
		)
{
	split_free(ar, t, file_version);
}

// ///////////////////////////////////////////////////////
//std unordered set
template<class Archive, class Value, class Hash, class Pred, class Allocator>
inline void save(
		Archive & ar,
		const std::unordered_set<Value, Hash, Pred, Allocator> &t,
		const unsigned int /* file_version */
		)
{
	boost::serialization::stl::save_collection<
			Archive,
			std::unordered_set<Value, Hash, Pred, Allocator>
	>(ar, t);
}

template<class Archive, class Value, class Hash, class Pred, class Allocator>
inline void load(
		Archive & ar,
		std::unordered_set<Value, Hash, Pred, Allocator> &t,
		const unsigned int /* file_version */
		)
{
	boost::serialization::stl::load_collection<
			Archive,
			std::unordered_set<Value, Hash, Pred, Allocator>,
			boost::serialization::stl::archive_input_set<
					Archive, std::unordered_set<Value, Hash, Pred, Allocator> >,
			boost::serialization::stl::no_reserve_imp<std::unordered_set<
					Value, Hash, Pred, Allocator
					>
			>
	>(ar, t);
}

//// split non-intrusive serialization function member into separate
//// non intrusive save/load member functions
template<class Archive, class Value, class Hash, class Pred, class Allocator>
inline void serialize(
		Archive & ar,
		std::unordered_set<Value, Hash, Pred, Allocator> &t,
		const unsigned int file_version
		)
{
	split_free(ar, t, file_version);
}

} // namespace serialization
} // namespace boost

#endif /* _c201d3cc58b1c333379b3fe2040cab25 */
