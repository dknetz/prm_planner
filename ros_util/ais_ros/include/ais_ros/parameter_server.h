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
//#ifndef PARAMETER_SERVER_H_
//#define PARAMETER_SERVER_H_
//
//#include <ais_ros/default_parameter_provider.h>
//
//#include <string>
//#include <ros/ros.h>
//#include <boost/any.hpp>
//#include <boost/serialization/serialization.hpp>
//
//namespace ais_ros
//{
//
//enum VerbosityLevel
//{
//	Off = 0,
//	Warn = 1,
//	Error = 2,
//	Info = 3,
//	Debug = 4
//};
//
///*!
// * \brief Getting values from parameter server.
// * This class is used for getting the parameters from
// * the parameter server
// */
//class ParameterServer
//{
//public:
//	/*!
//	 * Returns the singleton instance
//	 */
//	static ParameterServer* instance();
//
//	/*!
//	 * Loads the default configuration
//	 */
//	static void initialize(const DefaultParameterProvider& defaultConfig);
//
//	/*!
//	 * The method sets a value in the local cache and on the Parameter Server.
//	 * You can use bool, int, double and std::string for T
//	 *
//	 * \param param the name of the parameter
//	 * \value the new parameter value
//	 */
//	template<typename T>
//	void set(const std::string param,
//			T value)
//	{
//		if (config.count(param) == 0)
//		{
//			ROS_ERROR("ParameterServer: Ignoring invalid parameter: \"%s\"", param.c_str());
//			return;
//		}
//		try
//		{
//			boost::any_cast<T>(value); //fails if wrong param type
//		}
//		catch (boost::bad_any_cast& e)
//		{
//			ROS_ERROR("ParameterServer: Ignoring invalid parameter type: %s", e.what());
//			return;
//		}
//		config[param] = value;
//		setOnParameterServer(pre + param, value);
//	}
//
//	void setLocal(std::string name,
//			boost::any value);
//
//	/*!
//	 * The method returns a value from the local cache.
//	 * You can use bool, int, double and std::string for T
//	 *
//	 * \param param the name of the parameter
//	 * \return the parameter value
//	 */
//	template<typename T>
//	T get(const std::string param)
//	{
//		if (config.count(param) == 0)
//		{
//			ROS_FATAL("ParameterServer object queried for invalid parameter \"%s\"", param.c_str());
//		}
//
//		boost::any value = config[param];
//		try
//		{
//			return boost::any_cast<T>(value);
//		}
//		catch (boost::bad_any_cast& bac)
//		{
//			ROS_ERROR_STREAM("Bad cast: Requested data type <" << typeid(T).name() << "> for parameter '" << param << "'");
//		}
//	}
//
//	/*!
//	 * Provides access to the raw config data
//	 */
//	std::map<std::string, boost::any>& getConfigData()
//	{
//		return config;
//	}
//
//	/*!
//	 * Receives all values from the parameter server and store them
//	 * in the map 'config'.
//	 * Will be called in the constructor
//	 */
//	void getValues();
//
//private:
//	std::map<std::string, boost::any> config;
//
//	static ParameterServer* _instance;
//	std::string pre;
//	::ros::NodeHandle handle;
//
//	/*!
//	 * Default constructor
//	 * private, because of singleton
//	 */
//	ParameterServer();
//
//	/*!
//	 * Checks, whether the parameters are ok
//	 */
//	void checkValues();
//
//	/*!
//	 * Returns a value from the parameter server
//	 * Will only be used by getValue()
//	 *
//	 * \param param name of the parameter
//	 * \param def default value (get through defaultConfig())
//	 *
//	 * \return the parameter value
//	 */
//	template<typename T>
//	T getFromParameterServer(const std::string param,
//			T def)
//	{
//		T result;
//		handle.param(param, result, def);
//		return result;
//	}
//
//	template<typename T>
//	void setOnParameterServer(const std::string param,
//			T new_val)
//	{
//		handle.setParam(param, new_val);
//	}
//};
//
//inline std::string getParameterS(const std::string& name)
//{
//	return ParameterServer::instance()->get<std::string>(name);
//}
//
//inline int getParameterI(const std::string& name)
//{
//	return ParameterServer::instance()->get<int>(name);
//}
//
//inline double getParameterD(const std::string& name)
//{
//	return ParameterServer::instance()->get<double>(name);
//}
//
//inline bool getParameterB(const std::string& name)
//{
//	return ParameterServer::instance()->get<bool>(name);
//}
//
//inline std::vector<double> getParameterVD(const std::string& name)
//{
//	return ParameterServer::instance()->get<std::vector<double>>(name);
//}
//
//inline std::vector<int> getParameterVI(const std::string& name)
//{
//	return ParameterServer::instance()->get<std::vector<int>>(name);
//}
//
//template<class T>
//inline T getParameter(const std::string& name)
//{
//	return ParameterServer::instance()->get<T>(name);
//}
//
//} /* namespace ais_ros */
//
//#endif /* PARAMETER_SERVER_H_ */

#include <ais_log/log.h>

#define GET_PARAM(NODE_HANDLE, PARAM, VAR, PRINT)\
	if (!NODE_HANDLE.getParam(PARAM, VAR))\
	{\
		LOG_ERROR("missing parameter: " << PARAM);\
		return false;\
	}\
	else\
	{\
		LOG_DEBUG_COND(PRINT, "PARAM: " << PARAM << " = " << VAR);\
	}

#define GET_PARAM_VECTOR(NODE_HANDLE, PARAM, VAR, PRINT)\
	if (!NODE_HANDLE.getParam(PARAM, VAR))\
	{\
		LOG_ERROR("missing parameter: " << PARAM);\
		return false;\
	}\
	else\
	{\
		LOG_DEBUG_COND(PRINT, "PARAM: " << PARAM << " = vector of size " << VAR.size());\
	}
