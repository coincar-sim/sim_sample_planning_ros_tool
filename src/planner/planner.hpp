#pragma once

#include <dynamic_reconfigure/server.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// raises errors if put below
#include <simulation_utils/util_planner.hpp>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>

#include "sim_sample_planning_ros_tool/PlannerParameters.h"


namespace sim_sample_planning_ros_tool {

class Planner {
public:
    Planner(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher desiredMotionPub_;
    ros::Subscriber egoMotionSub_;
    ros::Subscriber predictedObjectsSub_;

    PlannerParameters params_;
    util_lanelet::lanelet_map_wrapper theMap_;
    util_planner::offset_planner planner_;

    int32_t currentLaneletId_ = 0;

    automated_driving_msgs::MotionState egoMotionState_;
    automated_driving_msgs::ObjectStateArray predictedObjectArray_;

    dynamic_reconfigure::Server<PlannerConfig> reconfigSrv_; // Dynamic reconfiguration service

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;


    void egoMotionCallback(const automated_driving_msgs::MotionState::ConstPtr& msg);
    void predictedObjectsCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);
    void reconfigureRequest(PlannerConfig&, uint32_t);

    void doPlanning();
};

} // namespace sim_sample_planning_ros_tool
