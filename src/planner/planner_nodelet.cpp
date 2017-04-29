#include "planner.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_planning_ros_tool {

class PlannerNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<Planner> m_;
};

void PlannerNodelet::onInit() {
    m_.reset(new Planner(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_planning_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_planning_ros_tool, PlannerNodelet, sim_sample_planning_ros_tool::PlannerNodelet, nodelet::Nodelet);
