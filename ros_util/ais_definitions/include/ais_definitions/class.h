/*
 * class.h
 *
 *  Created on: Dec 25, 2014
 *      Author: daniel
 */

#ifndef ROBOTIC_LIBS_DEFITIONS_CLASS_H_
#define ROBOTIC_LIBS_DEFITIONS_CLASS_H_

#define DELETE_VAR(var) if (var != NULL) { delete var; var = NULL; }
#define DELETE_ARRAY(var) if (var != NULL) { delete[] var; var = NULL; }
#define DELETE_ARRAY2(var) if (var != NULL) { delete[] var; }
#define DELETE_VECTOR(var) { for (auto it: var) { delete it; } var.clear(); }
#define DELETE_MAP(var) { for (auto it: var) { delete it.second; } var.clear(); }

#define PRIVATE_METHODS private
#define PRIVATE_VARIABLES private
#define PRIVATE_ENUMS private
#define PRIVATE_STRUCTS private
#define PRIVATE_OTHER private
#define PRIVATE_ENUMS private

#define PROTECTED_METHODS protected
#define PROTECTED_VARIABLES protected
#define PROTECTED_ENUMS protected
#define PROTECTED_STRUCTS protected
#define PROTECTED_OTHER protected
#define PROTECTED_ENUMS protected

#define PUBLIC_METHODS public
#define PUBLIC_VARIABLES public
#define PUBLIC_ENUMS public
#define PUBLIC_STRUCTS public
#define PUBLIC_OTHER public
#define PUBLIC_ENUM public

#define FRIENDS private
#define FRIEND friend class

#ifndef NULL
#define NULL 0
#endif

#define FORWARD_DECLARE_N(nSpace, className1) namespace nSpace { class className1; }
#define FORWARD_DECLARE_N2(nSpace, n2Space, className1) namespace nSpace { namespace n2Space { class className1; }}
#define FORWARD_DECLARE(className1) class className1;
#define FORWARD_DECLARE_STRUCT(className1) struct className1;

/**
 * SINGLETON
 */
#define SINGLETON_THREADED_HEADER(name) \
public:\
	static name* getInstance();\
protected:\
	name();\
	name(const name&);\
	void operator=(name const&);\
	static name* instance;\
	static boost::recursive_mutex __mutexSingleton;

#define SINGLETON_THREADED_SOURCE(name) \
name* name::instance = NULL; \
boost::recursive_mutex name::__mutexSingleton; \
name* name::getInstance() \
{\
	boost::recursive_mutex::scoped_lock lock(__mutexSingleton);\
	if (instance == NULL)\
	{\
		instance = new name;\
	}\
	return instance;\
}

#define SINGLETON_HEADER(name) \
public:\
	static name* getInstance();\
	static void deleteInstance();\
protected:\
	name();\
	name(const name&);\
	void operator=(name const&);\
	static name* instance;

#define SINGLETON_SOURCE(name) \
name* name::instance = NULL; \
name* name::getInstance() \
{\
	if (instance == NULL)\
	{\
		instance = new name;\
	}\
	return instance;\
}\
void name::deleteInstance() \
{\
	if (instance != NULL)\
	{\
		delete(instance);\
	}\
}

#endif /* ROBOTIC_LIBS_DEFITIONS_CLASS_H_ */
