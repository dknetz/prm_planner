/*
 * This file (collection.h) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 9, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef So2ws1lln3qWdQTNDHYl
#define So2ws1lln3qWdQTNDHYl

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <fcl_wrapper/collision_detection/common.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <Eigen/Geometry>
#include <unordered_map>
#include <Eigen/SparseCore>

namespace fcl_collision_detection
{

class Collection
{
public:
	typedef FCL_POINTER<Collection> Ptr;

	Collection();
	Collection(const std::string& name,
			const std::string& frame,
			const std::string& worldFrame);
	virtual ~Collection();

	virtual CollisionMatrix::Ptr getCollisionMatrix() = 0;
	virtual ObjectMap getObjects() = 0;
	const std::string& getFrame() const;
	void setFrame(const std::string& frame);
	const std::string& getName() const;
	void setName(const std::string& name);
	const std::string& getWorldFrame() const;
	void setWorldFrame(const std::string& worldFrame);

	/**
	 * Set transformation to world by hand
	 */
	virtual void setTransform(const Eigen::Affine3d& tf) = 0;

	/**
	 * Set transformation to world via
	 */
	virtual void updateTransformRosTF();

protected:
	mutable boost::recursive_mutex m_mutex;
	std::string m_name;
	std::string m_frame;
	std::string m_worldFrame;
};

typedef FCL_POINTER<Collection> CollectionPtr;

} /* namespace fcl_collision_detection */

#endif /* So2ws1lln3qWdQTNDHYl */
