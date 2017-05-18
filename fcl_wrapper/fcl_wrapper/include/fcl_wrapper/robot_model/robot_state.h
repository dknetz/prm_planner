/*
 * This file (robot_state.h) is part of the Scene Analyzer of Daniel Kuhner.
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
#ifndef N1zUVhgihwlz7JAE83u4
#define N1zUVhgihwlz7JAE83u4

#include <string>
#include <unordered_map>
#include <vector>

namespace fcl_robot_model
{

class RobotState
{
public:
	RobotState();
	RobotState(const std::vector<std::string>& jointNames);
	RobotState(const std::unordered_map<std::string, double>& value);
	virtual ~RobotState();

	void setJoint(const std::string& name,
			const double angle);
	double getJoint(const std::string& name);

	std::unordered_map<std::string, double>::const_iterator begin() const;
	std::unordered_map<std::string, double>::const_iterator end() const;
	std::unordered_map<std::string, double>::iterator begin();
	std::unordered_map<std::string, double>::iterator end();
	double& operator[](const std::string& joint);
	double getMaxAngularDistance(RobotState& other);

	void print() const;

	void clear();
	size_t size() const;
	void initAllToZero();

	static void interpolateStates(RobotState& s1,
			RobotState& s2,
			const double& t,
			RobotState& result);

	std::unordered_map<std::string, double> m_joints;
};

} /* namespace robot_model */

std::ostream& operator <<(std::ostream& stream,
		const fcl_robot_model::RobotState& state);

#endif /* N1zUVhgihwlz7JAE83u4 */
