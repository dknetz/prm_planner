/*
 * fcl_pointer.h
 *
 *  Created on: Sep 3, 2016
 *      Author: kuhnerd
 */

#ifndef FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_FCL_POINTER_H_
#define FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_FCL_POINTER_H_

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <fcl/config.h>
#include <memory>

template<typename T>
boost::shared_ptr<T> make_shared_ptr(std::shared_ptr<T>& ptr)
{
    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

template<typename T>
std::shared_ptr<T> make_shared_ptr(boost::shared_ptr<T>& ptr)
{
    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

#if FCL_MINOR_VERSION >= 5
#	define FCL_POINTER_NAMESPACE std
#	define CONVERT_TO_BOOST_POINTER(STD_PTR) make_shared_ptr(STD_PTR)
#	define CONVERT_TO_STD_POINTER(BOOST_PTR) make_shared_ptr(BOOST_PTR)
#else
# 	define FCL_POINTER_NAMESPACE boost
#	define CONVERT_TO_BOOST_POINTER(BOOST_PTR) BOOST_PTR
#	define CONVERT_TO_STD_POINTER(BOOST_PTR) BOOST_PTR
#endif

#define FCL_POINTER FCL_POINTER_NAMESPACE::shared_ptr
#define FCL_WEAK_POINTER FCL_POINTER_NAMESPACE::weak_ptr

#endif /* FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_FCL_POINTER_H_ */
