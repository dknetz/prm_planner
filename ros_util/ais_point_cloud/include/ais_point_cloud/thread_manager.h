///*
// * This file (thread_manager.h) is part of the "ros_util"-package of Daniel Kuhner.
// *
// * It is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * It is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
// *
// * created:		Jan 16, 2014
// * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
// *
// */
//#ifndef THREAD_MANAGER_H_
//#define THREAD_MANAGER_H_
//
//#include <ais_definitions/class.h>
//#include <ais_definitions/mutex.h>
//
//#include <boost/thread/mutex.hpp>
//#include <boost/thread.hpp>
//
//#include <unordered_map>
//
//#define THREAD_MANAGER_READ_LOCK() READ_LOCK_BASE(_mutex)
//#define THREAD_MANAGER_WRITE_LOCK() WRITE_LOCK_BASE(_mutex);
//#define THREAD_MANAGER_UPGRADE_LOCK() UPGRADE_LOCK_BASE(_mutex)
//#define THREAD_MANAGER_UPGRADEABLE_LOCK() UPGRADEABLE_LOCK_BASE(_mutex)
//
//namespace ros_util
//{
//
//class ThreadManager
//{
//SINGLETON_HEADER(ThreadManager)
//
//public:
//	virtual ~ThreadManager();
//
//	void add(boost::thread* thread,
//			const std::string& name);
//
//	bool waitForFinish();
//
//	bool doStop();
//	void finishAllThreads();
//
//	bool joinThread(const std::string& name);
//
//	bool finish(const std::string& name);
//	bool isFinished(const std::string& name);
//	static bool isFinished(boost::thread* thread);
//	void interrupt(const std::string& name);
//
//private:
//	std::unordered_map<std::string, boost::thread*> _threads;
//	mutable boost::shared_mutex _mutex;
//	bool _finishAllThreads;
//};
//
//} /* namespace ros_util */
//
//#endif /* THREAD_MANAGER_H_ */
