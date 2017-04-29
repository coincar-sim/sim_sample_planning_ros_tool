#include "planner_rviz.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "planner_rviz_node");

    sim_sample_planning_ros_tool::PlannerRviz planner_rviz(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
