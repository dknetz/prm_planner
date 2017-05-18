/*
 * stop_watch.h
 *
 *  Created on: Mar 17, 2015
 *      Author: kuhnerd
 */

#ifndef ROBOTIC_LIBS_UTIL_STOP_WATCH_H_
#define ROBOTIC_LIBS_UTIL_STOP_WATCH_H_

#include <ais_definitions/class.h>
#include <boost/chrono/chrono.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <unordered_map>

namespace ais_util
{

class StopWatch
{
SINGLETON_HEADER(StopWatch)

PUBLIC_METHODS:
	virtual ~StopWatch();

	void start(const std::string& name);
	double pause(const std::string& name);
	double stop(const std::string& name);
	void stopPrint(const std::string& name);
	void print(const std::string& name,
			double lastTime = 0); //lastTime is used internally

PRIVATE_STRUCTS:
	struct Statistics
	{
		Statistics() :
						accumulatedTime(0),
						runs(0)
		{
		}
		double accumulatedTime;
		int runs;
	};

PRIVATE_VARIABLES:
struct timespec ts = { 0, 0 };
	mutable boost::recursive_mutex m_mutex;
	std::unordered_map<std::string, boost::chrono::high_resolution_clock::time_point> m_startTimes;
	std::unordered_map<std::string, double> m_times;
	std::unordered_map<std::string, Statistics> m_statistics;
};

} /* namespace ais_util */

#endif /* ROBOTIC_LIBS_UTIL_STOP_WATCH_H_ */
