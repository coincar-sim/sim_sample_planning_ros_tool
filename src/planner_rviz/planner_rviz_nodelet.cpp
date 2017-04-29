#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "planner_rviz.hpp"

namespace sim_sample_planning_ros_tool {

class PlannerRvizNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PlannerRviz>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PlannerRviz> impl_;
};
} // namespace sim_sample_planning_ros_tool

PLUGINLIB_EXPORT_CLASS(sim_sample_planning_ros_tool::PlannerRvizNodelet, nodelet::Nodelet);
