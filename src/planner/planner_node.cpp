#include "planner.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "planner_node");

    sim_sample_planning_ros_tool::Planner planner(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
