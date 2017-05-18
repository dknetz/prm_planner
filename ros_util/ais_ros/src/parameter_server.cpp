///*
// * Copyright (c) 2013 Daniel Kuhner.
// * All rights reserved. This program and the accompanying materials
// * are made available under the terms of the GNU Public License v3.0
// * which accompanies this distribution, and is available at
// * http://www.gnu.org/licenses/gpl.html
// *
// * Contributors:
// *     Daniel Kuhner - kuhnerd@informatik.uni-freiburg.de
// */
//
//#include "ais_ros/parameter_server.h"
//
//#include <opencv/cv.h>
//
//namespace ais_ros
//{
//
//ParameterServer* ParameterServer::_instance = NULL;
//
//ParameterServer* ParameterServer::instance()
//{
//	assert(_instance != NULL); //you need to call initialize first
//	return _instance;
//}
//
//void ParameterServer::initialize(const DefaultParameterProvider& defaultConfig)
//{
//	if (_instance == NULL)
//	{
//		_instance = new ParameterServer;
//
//		for (auto it : defaultConfig.defaultConfig)
//		{
//			_instance->setLocal(it.first, it.second);
//		}
//
//		_instance->getValues();
//	}
//}
//
//ParameterServer::ParameterServer()
//{
//	pre = ros::this_node::getName();
//	pre += "/config/";
//}
//
//void ParameterServer::setLocal(std::string name,
//		boost::any value)
//{
//	config[name] = value;
//}
//
//void ParameterServer::getValues()
//{
//	std::map<std::string, boost::any>::const_iterator itr;
//	for (itr = config.begin(); itr != config.end(); ++itr)
//	{
//		std::string name = itr->first;
//		if (itr->second.type() == typeid(std::string))
//		{
//			config[name] = getFromParameterServer<std::string>(pre + name, boost::any_cast<std::string>(itr->second));
//			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<std::string>(itr->second));
//		}
//		else if (itr->second.type() == typeid(int))
//		{
//			config[name] = getFromParameterServer<int>(pre + name, boost::any_cast<int>(itr->second));
//			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
//		}
//		else if (itr->second.type() == typeid(double))
//		{
//			config[name] = getFromParameterServer<double>(pre + name, boost::any_cast<double>(itr->second));
//			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
//		}
//		else if (itr->second.type() == typeid(bool))
//		{
//			config[name] = getFromParameterServer<bool>(pre + name, boost::any_cast<bool>(itr->second));
//			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
//		}
//		else if (itr->second.type() == typeid(std::vector<double>))
//		{
//			config[name] = getFromParameterServer<std::vector<double>>(pre + name, boost::any_cast<std::vector<double>>(itr->second));
//		}
//		else if (itr->second.type() == typeid(std::vector<int>))
//		{
//			config[name] = getFromParameterServer<std::vector<int>>(pre + name, boost::any_cast<std::vector<int>>(itr->second));
//		}
//	}
//	checkValues();
//}
//
//void ParameterServer::checkValues()
//{
//}
//
//} /* namespace ais_ros */
