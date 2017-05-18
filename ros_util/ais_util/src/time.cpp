/*
 * time.cpp
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#include <boost/math/special_functions/round.hpp>
#include <ais_util/time.h>
#include <chrono>
#include <cmath>

namespace ais_util
{

Time::Time() :
				m_sec(0),
				m_nsec(0)
{
}

Time::~Time()
{
}

Time::Time(uint64_t sec,
		uint64_t nsec) :
				m_sec(sec),
				m_nsec(nsec)
{
}

double Time::toSec() const
{
	return (double) m_sec + 1e-9 * (double) m_nsec;
}

uint64_t Time::toNSec() const
{
	return (uint64_t) m_sec * 1000000000ull + (uint64_t) m_nsec;
}

void Time::fromNSec(uint64_t t)
{
	m_sec = (int32_t) (t / 1000000000);
	m_nsec = (int32_t) (t % 1000000000);

	normalizeSecNSec(m_sec, m_nsec);
}

void Time::fromSec(double t)
{
	m_sec = (uint32_t) floor(t);
	m_nsec = (uint32_t) boost::math::round((t - m_sec) * 1e9);
}

uint64_t Time::getNsec() const
{
	return m_nsec;
}

void Time::setNsec(uint64_t nsec)
{
	m_nsec = nsec;
}

uint64_t Time::getSec() const
{
	return m_sec;
}

void Time::setSec(uint64_t sec)
{
	m_sec = sec;
}

void Time::normalizeSecNSec(uint64_t& sec,
		uint64_t& nsec)
{
	uint64_t nsec_part = nsec % 1000000000UL;
	uint64_t sec_part = nsec / 1000000000UL;

	if (sec_part > UINT64_MAX)
	{
		throw std::runtime_error("Time is out of dual 64-bit range");
	}

	sec += sec_part;
	nsec = nsec_part;
}

Time Time::now()
{
	auto now = std::chrono::system_clock::now().time_since_epoch();
	u_int64_t sec = 0;
	u_int64_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
	normalizeSecNSec(sec, nsec);

	return Time(sec, nsec);
}

} /* namespace ais_util */

std::ostream& operator <<(std::ostream& stream,
		const ais_util::Time& time)
{
	stream << "Time: [sec: " << time.getSec() << ", nsec: " << time.getNsec() << "]";
	return stream;
}

