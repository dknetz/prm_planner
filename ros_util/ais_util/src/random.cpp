/*
 * random.cpp
 *
 *  Created on: Oct 5, 2015
 *      Author: kuhnerd
 */

#include <ais_util/random.h>
#include <random>

namespace ais_util
{

Random::Random()
{
}

Random::~Random()
{
}

double Random::uniform(double intervalA,
		double intervalB)
{
	static std::default_random_engine re((unsigned int) time(0));
	std::uniform_real_distribution<double> unif(intervalA, intervalB);
	return unif(re);
}

double Random::uniformProbability()
{
	return uniform(0, 1);
}

int Random::uniformIndex(int numberOfIndizes)
{
	static std::default_random_engine re((unsigned int) time(0));
	std::uniform_int_distribution<int> unif(0, numberOfIndizes - 1);
	return unif(re);
}
} /* namespace ais_util */
