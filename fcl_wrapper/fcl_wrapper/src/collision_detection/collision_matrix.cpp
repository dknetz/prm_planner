/*
 * This file (collision_matrix.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

namespace fcl_collision_detection
{

CollisionMatrix::CollisionMatrix()
{
	init();
}

CollisionMatrix::CollisionMatrix(const std::string& o1,
		const std::vector<std::string>& o2)
{
	init();
	for (auto o : o2)
		set(o1, o, true);
}

CollisionMatrix::~CollisionMatrix()
{
}

void CollisionMatrix::init()
{
	m_collisionMatrix.clear();
}

void CollisionMatrix::set(const std::string& link1,
		const std::string& link2,
		bool canHaveCollision)
{
	if (canHaveCollision)
	{
		auto range = m_collisionMatrix.equal_range(link1);
		bool found = false;
		for (auto it = range.first; it != range.second; ++it)
		{
			if (it->second == link2)
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			m_collisionMatrix.insert(std::make_pair(link1, link2));
			m_collisionMatrix.insert(std::make_pair(link2, link1));
		}
	}
}

void CollisionMatrix::setDoAllCollisionChecks(const std::string& object,
		bool canHaveCollision)
{
	if (canHaveCollision)
	{
		m_checkAllways[object] = true;
	}
}

void CollisionMatrix::addOtherCollisionMatrix(const CollisionMatrix::Ptr& collisionMatrix)
{
	m_collisionMatrix.insert(collisionMatrix->m_collisionMatrix.begin(), collisionMatrix->m_collisionMatrix.end());
}

//void CollisionMatrix::setPrefix(const std::string& prefix,
//		const std::list<std::string>& exclude)
//{
//	CM newCM;
//	std::string first, second;
//
//	for (auto& it : m_collisionMatrix)
//	{
//		first = it.first;
//		second = it.second;
//
//		if (std::find(exclude.begin(), exclude.end(), first) == exclude.end())
//			first = prefix + "/" + first;
//
//		if (std::find(exclude.begin(), exclude.end(), second) == exclude.end())
//			second = prefix + "/" + second;
//
//		newCM.insert(std::make_pair(first, second));
//	}
//
//	m_prefix = prefix;
//	m_collisionMatrix = newCM;
//}

bool CollisionMatrix::canCollide(const std::string& link1,
		const std::string& link2)
{
//	std::string l1 = m_prefix.empty() ? link1 : (m_prefix + "/" + link1);
//	std::string l2 = m_prefix.empty() ? link2 : (m_prefix + "/" + link2);

//	LOG_INFO("Collision check between " << link1 << " " << link2);

	auto range = m_collisionMatrix.equal_range(link1);

	if (CHECK_MAP(m_checkAllways, link1) || CHECK_MAP(m_checkAllways, link2))
	{
		return true;
	}

	for (auto it = range.first; it != range.second; ++it)
	{
		if (it->second == link2)
		{
			return true;
		}
	}
	return false;
}

void CollisionMatrix::print()
{
	for (auto& it : m_collisionMatrix)
	{
		LOG_INFO(it.first << " <-> " << it.second << " have collisions");
	}
}

bool CollisionMatrix::write(const std::string& file) const
		{
	std::ofstream f;
	f.open(file);
	if (!f.is_open())
	{
		LOG_ERROR("I cannot open the file!");
		return false;
	}

	for (auto& it : m_collisionMatrix)
	{
		f << it.first << " " << it.second << "\n";
	}

	f.close();

	return true;
}

bool CollisionMatrix::load(const std::string& file,
		CollisionMatrix::Ptr& matrix)
{
	matrix.reset(new CollisionMatrix);

	std::ifstream f;
	f.open(file);

	if (!f.is_open())
	{
		LOG_ERROR("I cannot open the file!");
		return false;
	}

	std::string line;
	while (std::getline(f, line))
	{
		std::istringstream iss(line);
		std::vector<std::string> tokens { std::istream_iterator<std::string> { iss },
				std::istream_iterator<std::string> { } };

		if (tokens.size() != 2)
		{
			LOG_ERROR("Error occured while reading collision matrix from file in line: '" << line << "'");
			return false;
		}

		matrix->set(tokens[0], tokens[1], true);
		matrix->set(tokens[1], tokens[0], true);
	}

	return true;
}

bool CollisionMatrix::checkAllCollisions()
{
	return m_collisionMatrix.empty();
}

} /* namespace world */

