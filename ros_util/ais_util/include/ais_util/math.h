/*
 * math.h
 *
 *  Created on: Sep 9, 2015
 *      Author: kuhnerd
 */

#ifndef SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_MATH_H_
#define SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_MATH_H_
#include <Eigen/Geometry>

namespace ais_util
{
template<typename T> int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

inline Eigen::Affine3d combinePositionAndOrientation(const Eigen::Vector3d& pos,
		const Eigen::Vector3d& orientation)
{
	Eigen::Affine3d result;
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ());
	result.linear() = rot;
	result.translation() = pos;
	return result;
}

inline double normalizeAngle(const double& angle)
{
	static const double PI2 = 2.0 * M_PI;
	if (angle > M_PI)
	{
		return angle - PI2;
	}
	else if (angle < -M_PI)
	{
		return angle + PI2;
	}
	return angle;
}

#ifndef DEG2RAD
#define DEG2RAD(x) (x/180.0*M_PI)
#endif

#ifndef DEG2RAD
#define RAD2DEG(x) (x/M_PI*180.0)
#endif

}

#endif /* SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_MATH_H_ */
