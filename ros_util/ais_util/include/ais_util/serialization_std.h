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
#ifndef _ab50758d31c9d85330981624f2807a54
#define _ab50758d31c9d85330981624f2807a54

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

#include <unordered_map>
#include <unordered_set>
#include <iosfwd>

#include <ais_log/log.h>

namespace boost
{
namespace serialization
{

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

#endif /* _ab50758d31c9d85330981624f2807a54 */
