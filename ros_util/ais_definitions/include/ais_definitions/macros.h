/*
 * macros.h
 *
 *  Created on: Dec 25, 2014
 *      Author: daniel
 */

#ifndef ROBOTIC_LIBS_DEFITIONS_MACROS_H_
#define ROBOTIC_LIBS_DEFITIONS_MACROS_H_

#define FOR_WIDTH_HEIGHT for (int x = 0; x < width; ++x) for (int y = 0; y < height; ++y)
#define FOR_PARAM(width, height) for (int x = 0; x < width; ++x) for (int y = 0; y < height; ++y)
#define FOR_VECTOR(counter, vector) for (size_t counter = 0; counter < vector.size(); ++counter)
#define FOR_COUNTER(counter, size) for (size_t counter = 0; counter < size; ++counter)
#define FOR_CLOUD_NORMAL_CONST(c) for(pcl::PointCloud<pcl::Normal>::const_iterator it = c->begin(); it != c->end(); ++it)
#define FOR_CLOUD_CONST(cloud) for(MyPointCloud::const_iterator it = cloud->begin(), itEnd = cloud->end(); it != itEnd; ++it)

#define CHECK_MAP(MAP, KEY) (MAP.find(KEY) != MAP.end())
#define IF_CHECK_MAP_VAR(MAP, KEY, VAR_NAME) auto VAR_NAME = MAP.find(KEY); if(VAR_NAME != MAP.end())
#define CHECK_VECTOR(VECTOR, INDEX) (INDEX >= 0 && INDEX < VECTOR.size())

#define STRINGIFY(s) STRINGIFY_HELP(s)
#define STRINGIFY_HELP(s) #s

#endif /* ROBOTIC_LIBS_DEFITIONS_MACROS_H_ */
