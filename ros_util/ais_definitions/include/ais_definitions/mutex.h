/*
 * mutex.h
 *
 *  Created on: Dec 25, 2014
 *      Author: daniel
 */

#ifndef ROBOTIC_LIBS_DEFITIONS_MUTEX_H_
#define ROBOTIC_LIBS_DEFITIONS_MUTEX_H_

#define READ_LOCK_BASE(name) boost::shared_lock<boost::shared_mutex> __base_lock(name);
#define READ_LOCK_BASE_TRY(name, ...) boost::shared_lock<boost::shared_mutex> __base_lock(name, boost::try_to_lock);\
	if (!__base_lock) {\
		return __VA_ARGS__;\
	}
#define WRITE_LOCK_BASE(name) boost::unique_lock<boost::shared_mutex> __base_lock(name);
#define UPGRADEABLE_LOCK_BASE(name) boost::upgrade_lock<boost::shared_mutex> __base_lock2(name);
#define UPGRADE_LOCK_BASE(name) boost::upgrade_to_unique_lock<boost::shared_mutex> __base_exclusive_lock(__base_lock2);

#endif /* ROBOTIC_LIBS_DEFITIONS_MUTEX_H_ */
