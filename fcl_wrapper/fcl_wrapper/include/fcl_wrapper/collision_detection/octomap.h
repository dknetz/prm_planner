/*
 * This file (render_octomap.h) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Jan 13, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef rjJyZ0KLqXRCkzNy4HHe
#define rjJyZ0KLqXRCkzNy4HHe

#include <boost/shared_ptr.hpp>
#include <fcl/octree.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <octomap/OcTree.h>

#include <unordered_map>

namespace fcl_collision_detection
{

class Octomap: public PhysicalObject
{
public:
	Octomap(boost::shared_ptr<octomap::OcTree> octomap,
			const std::string& name,
			const std::string& frame,
			const std::string& worldFrame);
	virtual ~Octomap();

	void updateOctomap(boost::shared_ptr<octomap::OcTree> octomap);

	virtual void initFCLModel();

private:
	boost::shared_ptr<octomap::OcTree> m_octree;
	FCL_POINTER<fcl::OcTree> m_fclOctree;
};

} /* namespace fcl_collision_detection */

#endif /* rjJyZ0KLqXRCkzNy4HHe */
