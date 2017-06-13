/*
 * This file (collision_matrix.h) is part of the Scene Analyzer of Daniel Kuhner.
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
#ifndef SwRoe82wda6y9ggVUOCP
#define SwRoe82wda6y9ggVUOCP

#include <boost/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <list>
#include <unordered_map>

namespace fcl_collision_detection
{

class CollisionMatrix
{
public:
	typedef std::unordered_multimap<std::string, std::string> CM;
	typedef FCL_POINTER<CollisionMatrix> Ptr;

	CollisionMatrix();
	CollisionMatrix(const std::string& o1, const std::vector<std::string>& o2);
	virtual ~CollisionMatrix();

	void init();

	void addOtherCollisionMatrix(const CollisionMatrix::Ptr& collisionMatrix);
//	void setPrefix(const std::string& prefix,
//			const std::list<std::string>& exclude); //without /

	void set(const std::string& link1,
			const std::string& link2,
			bool canHaveCollision);

	void setDoAllCollisionChecks(const std::string& object,
			bool canHaveCollision);

	bool canCollide(const std::string& link1,
			const std::string& link2);

	void print();

	bool checkAllCollisions();

	bool write(const std::string& file) const;
	static bool load(const std::string& file,
			CollisionMatrix::Ptr& matrix);

private:
	CM m_collisionMatrix;
//	std::string m_prefix;
	std::unordered_map<std::string, bool> m_checkAllways;
};

} /* namespace fcl_collision_detection */

#endif /* SwRoe82wda6y9ggVUOCP */
