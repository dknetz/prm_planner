/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: box.h
 */

#ifndef HF6E12BEE_2A93_4138_95FE_40AD95951D08
#define HF6E12BEE_2A93_4138_95FE_40AD95951D08

#include <fcl_wrapper/collision_detection/physical_object.h>

namespace fcl_collision_detection
{

class Box: public PhysicalObject
{
public:
	Box(double sizeX,
			double sizeY,
			double sizeZ,
			const std::string& name,
			const std::string& frame,
			const std::string& worldFrame);

	virtual ~Box();

	virtual void initFCLModel();

private:
	double m_sizeX;
	double m_sizeY;
	double m_sizeZ;
};

} /* namespace fcl_collision_detection */

#endif /* HF6E12BEE_2A93_4138_95FE_40AD95951D08 */
