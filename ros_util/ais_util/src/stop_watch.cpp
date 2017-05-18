/*
 * stop_watch.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: kuhnerd
 */

#include <ais_util/stop_watch.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <boost/chrono/chrono.hpp>

namespace ais_util
{

SINGLETON_SOURCE(StopWatch)

StopWatch::StopWatch()
{
}

StopWatch::~StopWatch()
{
}

void StopWatch::start(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (CHECK_MAP(m_startTimes, name))
	{
		LOG_WARNING("StopWatch '" << name << "' already running!");
	}

	m_startTimes[name] = boost::chrono::high_resolution_clock::now();
}

double StopWatch::pause(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	boost::chrono::duration<double> diff = boost::chrono::high_resolution_clock::now() - m_startTimes[name];
	if (CHECK_MAP(m_times, name))
	{
		m_times[name] += diff.count();
	}
	else
	{
		m_times[name] = diff.count();
	}

	m_startTimes.erase(name);

	return m_times[name];
}

double StopWatch::stop(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	double time = pause(name);
	m_statistics[name].accumulatedTime += time;
	++m_statistics[name].runs;
	m_times[name] = 0;
	return time;
}

void StopWatch::stopPrint(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	print(name, stop(name));
}

void StopWatch::print(const std::string& name,
		double lastTime)
{
	double average = (m_statistics[name].accumulatedTime / m_statistics[name].runs);
	double sum = m_statistics[name].accumulatedTime;
	std::string sumUnit = "s";
	if (sum > 60)
	{
		sum /= 60;
		sumUnit = "m";
	}
	if (sum > 60)
	{
		sum /= 60;
		sumUnit = "h";
	}

	if (lastTime == 0)
	{
		LOG_INFO("StopWatch '" << name << "': Average: " <<
				average << "s (" << (1.0 / average) << "hz), Sum: " << sum << sumUnit);
	}
	else
	{
		LOG_INFO("StopWatch '" << name << "' stopped at: " << lastTime << "s, Average: " <<
				average << "s (" << (1.0 / average) << "hz), Sum: " << sum << sumUnit);
	}
}

} /* namespace ais_util */

