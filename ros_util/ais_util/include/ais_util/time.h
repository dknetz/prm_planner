/*
 * time.h
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#ifndef SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_TIME_H_
#define SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_TIME_H_

#include <ais_definitions/class.h>
#include <limits.h>
#include <stdint.h>
#include <ostream>

namespace ais_util
{

class Time
{
PUBLIC_METHODS:
	Time();
	Time(uint64_t sec,
			uint64_t nsec);
	virtual ~Time();

	double toSec() const;
	void fromSec(double t);

	uint64_t toNSec() const;
	void fromNSec(uint64_t t);

	static void normalizeSecNSec(uint64_t& sec,
			uint64_t& nsec);
	static Time now();

	uint64_t getNsec() const;
	void setNsec(uint64_t nsec);
	uint64_t getSec() const;
	void setSec(uint64_t sec);

PRIVATE_VARIABLES:
	uint64_t m_sec;
	uint64_t m_nsec;
};

} /* namespace ais_util */

std::ostream& operator <<(std::ostream& stream,
		const ais_util::Time& time);

#endif /* SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_TIME_H_ */
