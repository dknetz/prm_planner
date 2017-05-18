/*
 * This file (robot_state.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 16, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#include <math.h>

#include <ais_log/log.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <sstream>

namespace fcl_robot_model
{

RobotState::RobotState()
{
}

RobotState::RobotState(const std::vector<std::string>& jointNames)
{
	for (auto& it : jointNames)
	{
		m_joints[it] = 0;
	}
}

RobotState::RobotState(const std::unordered_map<std::string, double>& value) :
				m_joints(value)
{
}

RobotState::~RobotState()
{
}

void RobotState::setJoint(const std::string& name,
		const double angle)
{
	m_joints[name] = angle;
}

double RobotState::getJoint(const std::string& name)
{
	return m_joints[name];
}

void RobotState::clear()
{
	m_joints.clear();
}

std::unordered_map<std::string, double>::const_iterator RobotState::begin() const
{
	return m_joints.begin();
}

std::unordered_map<std::string, double>::const_iterator RobotState::end() const
{
	return m_joints.end();
}

std::unordered_map<std::string, double>::iterator RobotState::begin()
{
	return m_joints.begin();
}

std::unordered_map<std::string, double>::iterator RobotState::end()
{
	return m_joints.end();
}

double& RobotState::operator [](const std::string& joint)
{
	return m_joints[joint];
}

void RobotState::print() const
{
	std::stringstream st;
	st << "RobotState:\n";
	for (auto& it : m_joints)
	{
		st << "\tJoint \"" << it.first << "\": " << it.second << "\n";
	}
	LOG_INFO(st.str());
}

size_t RobotState::size() const
{
	return m_joints.size();
}

void RobotState::initAllToZero()
{
	for (auto& it : m_joints)
	{
		it.second = 0;
	}
}

double RobotState::getMaxAngularDistance(RobotState& other)
{
	double max = 0;
	for (auto& it : m_joints)
	{
		double diff = fabs(it.second - other.m_joints[it.first]);
		if (diff > max)
		{
			max = diff;
		}
	}
	return max;
}

void RobotState::interpolateStates(RobotState& s1,
		RobotState& s2,
		const double& t,
		RobotState& result)
{
	for (auto& it : s1)
	{
		result[it.first] = it.second + t * (s2[it.first] - it.second);
	}
}

} /* namespace robot_model */

std::ostream& operator <<(std::ostream& stream,
		const fcl_robot_model::RobotState& state)
{
	stream << "RobotState: [";
	for (auto& it : state)
	{
		stream << it.first << "=" << it.second << ",";
	}
	stream << "]";

	return stream;
}

