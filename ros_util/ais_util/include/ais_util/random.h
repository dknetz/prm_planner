/*
 * random.h
 *
 *  Created on: Oct 5, 2015
 *      Author: kuhnerd
 */

#ifndef SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_RANDOM_H_
#define SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_RANDOM_H_

namespace ais_util
{

class Random
{
public:
	Random();
	virtual ~Random();

	static double uniform(double intervalA,
			double intervalB);
	static double uniformProbability();

	static int uniformIndex(int numberOfIndizes);
};

} /* namespace ais_util */

#endif /* SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_RANDOM_H_ */
