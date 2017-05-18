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
#ifndef _a4dade5a59f5cc05cfc8a9b88afa45d2
#define _a4dade5a59f5cc05cfc8a9b88afa45d2

#include <ais_util/serialization_eigen.h>
#include <kdl/jntarray.hpp>

#include <ais_log/log.h>

namespace boost
{
namespace serialization
{

//for KDL::JntArray
template<class Archive>
inline void serialize(
		Archive & ar,
		KDL::JntArray & t,
		const unsigned int file_version
		)
{
	ar & t.data;
}

} // namespace serialization
} // namespace boost

#endif /* _a4dade5a59f5cc05cfc8a9b88afa45d2 */
