#pragma once

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include "automated_driving_msgs/MotionState.h"
#include "automated_driving_msgs/ObjectStateArray.h"
#include "simulation_only_msgs/DeltaTrajectoryWithID.h"

#include "sim_sample_planning_ros_tool/PlannerRvizParameters.h"

namespace sim_sample_planning_ros_tool {

class PlannerRviz {

    using Parameters = PlannerRvizParameters;
    using Config = PlannerRvizConfig;

    using Msg = std_msgs::Header;

public:
    PlannerRviz(ros::NodeHandle, ros::NodeHandle);

private:
    void egoMotionCallback(const automated_driving_msgs::MotionState::ConstPtr& msg);
    void objectsStateCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);
    void desiredMotionSubCallback(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& msg);

    ros::Subscriber desiredMotionSub_;
    ros::Subscriber egoMotionSub_;
    ros::Subscriber objectsStateSub_;
    ros::Publisher navMsgsPubEgoView_;
    ros::Publisher navMsgsPubGroundTruth_;

    Parameters params_;

    automated_driving_msgs::MotionState egoMotionStateEgoView;
    automated_driving_msgs::MotionState egoMotionStateGroundTruth;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace sim_sample_planning_ros_tool
