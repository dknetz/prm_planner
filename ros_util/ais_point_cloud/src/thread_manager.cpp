///*
// * This file (thread_manager.cpp) is part of the "ros_util"-package of Daniel Kuhner.
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
//
//#include <ros_util/thread_manager.h>
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <ros/ros.h>
//#include <ais_log/log.h>
//
//namespace ros_util
//{
//
//SINGLETON_SOURCE(ThreadManager)
//
//ThreadManager::ThreadManager() :
//				_finishAllThreads(false)
//{
//}
//
//ThreadManager::~ThreadManager()
//{
//	waitForFinish();
//}
//
//void ThreadManager::add(boost::thread* thread,
//		const std::string& name)
//{
//	THREAD_MANAGER_WRITE_LOCK();
//	LOG_INFO("add thread " << name.c_str());
//
//	if (_threads.find(name) != _threads.end())
//	{
//		LOG_ERROR("Thread cannot be added, since a thread with the same name already exists");
//		return;
//	}
//
//	_threads[name] = thread;
//}
//
//bool ThreadManager::waitForFinish()
//{
//	finishAllThreads();
//
//	THREAD_MANAGER_UPGRADEABLE_LOCK();
//	std::stringstream st;
//	size_t size = 0;
//
//	for (auto thread : _threads)
//	{
//		st << thread.first;
//		if (size < _threads.size() - 1)
//			st << ", ";
//		++size;
//	}
//
//	LOG_INFO("Wait for " << _threads.size() << " threads: " << st.str().c_str());
//
//	while (!_threads.empty())
//	{
//		std::stringstream st;
//
//		for (auto thread = _threads.begin(); thread != _threads.end();)
//		{
//#if BOOST_VERSION >= 105400
//			if (!thread->second->try_join_for(boost::chrono::milliseconds(2)))
//			#else
//			if (!thread->second->timed_join(boost::posix_time::milliseconds(2)))
//#endif
//			{
//				++thread;
//				continue;
//			}
//			else
//			{
//				THREAD_MANAGER_UPGRADE_LOCK();
//				delete thread->second;
//				_threads.erase(thread++);
//			}
//		}
//
//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
//	}
//
//	LOG_INFO("There are no threads to be waited for!");
//
//	return true;
//}
//
//bool ThreadManager::doStop()
//{
//	THREAD_MANAGER_READ_LOCK();
//	return _finishAllThreads || !ros::ok();
//}
//
//void ThreadManager::finishAllThreads()
//{
//	THREAD_MANAGER_WRITE_LOCK();
//	_finishAllThreads = true;
//}
//
//bool ThreadManager::joinThread(const std::string& name)
//{
//	while (true)
//	{
//		{
//			THREAD_MANAGER_WRITE_LOCK();
//
//			if (_threads.find(name) != _threads.end())
//			{
//#if BOOST_VERSION >= 105400
//				if (_threads[name]->try_join_for(boost::chrono::milliseconds(2)))
//#else
//				if (_threads[name]->timed_join(boost::posix_time::milliseconds(2)))
//#endif
//				{
//					return true;
//				}
//			}
//			else
//			{
//				return false;
//			}
//		}
//
//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
//	}
//
//	return false;
//}
//
//bool ThreadManager::finish(const std::string& name)
//{
//	{
//		THREAD_MANAGER_READ_LOCK();
//
//		if (_threads.find(name) == _threads.end())
//		{
//			return true;
//		}
//	}
//
//	if (joinThread(name))
//	{
//		THREAD_MANAGER_WRITE_LOCK();
//		delete _threads[name];
//		_threads.erase(name);
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//bool ThreadManager::isFinished(const std::string& name)
//{
//	THREAD_MANAGER_WRITE_LOCK();
//
//	if (_threads.find(name) == _threads.end())
//	{
//		return true;
//	}
//
//#if BOOST_VERSION >= 105400
//	return _threads[name]->try_join_for(boost::chrono::milliseconds(2));
//#else
//	return _threads[name]->timed_join(boost::posix_time::milliseconds(2));
//#endif
//}
//
//bool ThreadManager::isFinished(boost::thread* thread)
//{
//#if BOOST_VERSION >= 105400
//	return thread->try_join_for(boost::chrono::milliseconds(2));
//#else
//	return thread->timed_join(boost::posix_time::milliseconds(2));
//#endif
//}
//
//void ThreadManager::interrupt(const std::string& name)
//{
//	THREAD_MANAGER_WRITE_LOCK();
//
//	if (_threads.find(name) == _threads.end())
//	{
//		return;
//	}
//
//	_threads[name]->interrupt();
//}
//
//}
///* namespace ros_util */
//
