/*
 * object_manager.h
 *
 *  Created on: Sep 10, 2016
 *      Author: kuhnerd
 */

#ifndef H5BB7C0E7_1A4D_40AD_A3B7_1965A28A82E5
#define H5BB7C0E7_1A4D_40AD_A3B7_1965A28A82E5
#include <prm_planner/objects/graspable_object.h>
#include <ais_definitions/class.h>
#include <boost/thread/mutex.hpp>
#include <octomap/OcTree.h>
#include <prm_planner/util/parameters.h>
#include <unordered_map>

namespace prm_planner
{

class ObjectManager
{
SINGLETON_HEADER(ObjectManager)

public:
	virtual ~ObjectManager();

	void loadObjects(std::unordered_map<std::string, parameters::ObjectConfig>& params,
			const parameters::DroppingConfig& droppingConfig,
			double octomapResolution,
			const std::string& frame);
	void addObject(boost::shared_ptr<GraspableObject> object);

	boost::shared_ptr<GraspableObject> getObject(const std::string& name);

	void publish();
	void updatePosesFromTF();
	std::unordered_map<std::string, boost::shared_ptr<GraspableObject> >& getObjects();
	void setNeglectObject(const std::vector<std::string>& neglectedObjects);

	//returns true, if world has changed
//	bool setRemoveObject(const std::string& name,
//			bool remove);
//	void toggleRemoveObject(const std::string& name);
//	void updateOctomapWithObjects(boost::shared_ptr<octomap::OcTree>& octomap);

private:
	mutable boost::mutex m_mutex;
	std::unordered_map<std::string, boost::shared_ptr<GraspableObject>> m_objects;
//	std::unordered_map<std::string, bool> m_remove;
};

} /* namespace prm_planner */

#endif /* H5BB7C0E7_1A4D_40AD_A3B7_1965A28A82E5 */
