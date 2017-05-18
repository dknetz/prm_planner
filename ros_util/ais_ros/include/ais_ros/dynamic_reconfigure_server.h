/*
 * This file (dynamic_reconfigure_server.h) is part of the "ais_ros"-package of Daniel Kuhner.
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
 * created:		Jul 4, 2014
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef DYNAMIC_RECONFIGURE_SERVER_H_
#define DYNAMIC_RECONFIGURE_SERVER_H_

#include <dynamic_reconfigure/server.h>

namespace ais_ros
{

template<typename DynamicReconfigure>
class DynamicReconfigureServer
{
public:
	DynamicReconfigureServer();
	virtual ~DynamicReconfigureServer();

	void callback(DynamicReconfigure& config,
			uint32_t level);

	DynamicReconfigure& get();

private:
	DynamicReconfigure config;
	dynamic_reconfigure::Server<DynamicReconfigure> server;
	boost::function<void(DynamicReconfigure &, uint32_t level)> callbackType;
};

} /* namespace ais_ros */

template<typename DynamicReconfigure>
inline ais_ros::DynamicReconfigureServer<DynamicReconfigure>::DynamicReconfigureServer()
{
	callbackType = boost::bind(&DynamicReconfigureServer<DynamicReconfigure>::callback, this, _1, _2);
	server.setCallback(callbackType);
}

template<typename DynamicReconfigure>
inline ais_ros::DynamicReconfigureServer<DynamicReconfigure>::~DynamicReconfigureServer()
{
}

template<typename DynamicReconfigure>
inline void ais_ros::DynamicReconfigureServer<DynamicReconfigure>::callback(DynamicReconfigure& config,
		uint32_t level)
{
	this->config = config;
}

template<typename DynamicReconfigure>
inline DynamicReconfigure& ais_ros::DynamicReconfigureServer<DynamicReconfigure>::get()
{
	return config;
}

#endif /* DYNAMIC_RECONFIGURE_SERVER_H_ */
