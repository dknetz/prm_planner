/*
 * hashes.h
 *
 *  Created on: Dec 25, 2014
 *      Author: daniel
 */

#ifndef ROBOTIC_LIBS_DEFITIONS_HASHES_H_
#define ROBOTIC_LIBS_DEFITIONS_HASHES_H_

#include <boost/functional/hash.hpp>
#include <Eigen/Core>

namespace ais_definitions
{
struct HasherEigen3d
{
	inline double round6(const double& value) const
			{
		return floor(value * 1000000.0 + 0.5) / 1000000.0;
	}

	std::size_t operator()(const Eigen::Vector3d& k) const
			{
		using boost::hash_value;
		using boost::hash_combine;

		// Start with a hash value of 0    .
		std::size_t seed = 0;

		// Modify 'seed' by XORing and bit-shifting in
		// one member of 'Key' after the other:
		hash_combine(seed, hash_value(round6(k.x())));
		hash_combine(seed, hash_value(round6(k.y())));
		hash_combine(seed, hash_value(round6(k.z())));

		// Return the result.
		return seed;
	}
};

struct HasherEigen3i
{
	std::size_t operator()(const Eigen::Vector3i& k) const
			{
		using boost::hash_value;
		using boost::hash_combine;

		// Start with a hash value of 0    .
		std::size_t seed = 0;

		// Modify 'seed' by XORing and bit-shifting in
		// one member of 'Key' after the other:
		hash_combine(seed, hash_value(k.x()));
		hash_combine(seed, hash_value(k.y()));
		hash_combine(seed, hash_value(k.z()));

		// Return the result.
		return seed;
	}
};

struct HasherStringPair
{
	std::size_t operator()(const std::pair<std::string, std::string>& k) const
			{
		using boost::hash_value;
		using boost::hash_combine;

		// Start with a hash value of 0    .
		std::size_t seed = 0;

		// Modify 'seed' by XORing and bit-shifting in
		// one member of 'Key' after the other:
		hash_combine(seed, hash_value(k.first));
		hash_combine(seed, hash_value(k.second));

		// Return the result.
		return seed;
	}
};
}

#endif /* ROBOTIC_LIBS_DEFITIONS_HASHES_H_ */
