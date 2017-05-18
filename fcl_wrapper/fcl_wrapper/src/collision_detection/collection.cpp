/*
 * This file (collection.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
#include <ais_ros/ros_base_interface.h>
#include <fcl_wrapper/collision_detection/collection.h>

namespace fcl_collision_detection
{

Collection::Collection()
{
}

Collection::Collection(const std::string& name,
		const std::string& frame,
		const std::string& worldFrame) :
				m_name(name),
				m_frame(frame),
				m_worldFrame(worldFrame)
{
}

Collection::~Collection()
{
}

const std::string& Collection::getFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_frame;
}

void Collection::setFrame(const std::string& frame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_frame = frame;
}

const std::string& Collection::getName() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_name;
}

void Collection::setName(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_name = name;
}

const std::string& Collection::getWorldFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_worldFrame;
}

void Collection::setWorldFrame(const std::string& worldFrame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_worldFrame = worldFrame;
}

void Collection::updateTransformRosTF()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	setTransform(ais_ros::RosBaseInterface::getRosTransformation(m_frame, m_worldFrame));
}

} /* namespace world */
