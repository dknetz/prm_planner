/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: sphere.h
 */

#ifndef H55B49782_9D9B_4269_B739_DC322A4AD332
#define H55B49782_9D9B_4269_B739_DC322A4AD332

#include <fcl_wrapper/collision_detection/physical_object.h>

namespace fcl_collision_detection
{

class Sphere: public PhysicalObject
{
public:
	Sphere(double radius,
			const std::string& name,
			const std::string& frame,
			const std::string& worldFrame);
	virtual ~Sphere();

	virtual void initFCLModel();

private:
	double m_radius;
};

} /* namespace fcl_collision_detection */

#endif /* H55B49782_9D9B_4269_B739_DC322A4AD332 */
