#ifndef ROBOTIC_LIBS_UTIL_THREAD_H_
#define ROBOTIC_LIBS_UTIL_THREAD_H_

#include <boost/atomic.hpp>

namespace ais_util
{

template<typename T>
void atomicMax(boost::atomic<T>& maximumValue,
		T const& value) noexcept
{
	T prev_value = maximumValue;
	while (prev_value < value &&
			!maximumValue.compare_exchange_weak(prev_value, value))
		;
}

}

#endif /* ROBOTIC_LIBS_UTIL_THREAD_H_ */
