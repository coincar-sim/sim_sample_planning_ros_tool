/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <memory>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <util_geometry_msgs/util_geometry_msgs.hpp>
#include <lanelet2_core/primitives/Lanelet.h>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectStateArray.h>

#include <sim_sample_planning_ros_tool/PlannerInterface.h>

#include "util_planner.hpp"

namespace sim_sample_planning_ros_tool {

class Planner {
public:
    Planner(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher desiredMotionPub_;
    ros::Subscriber egoMotionSub_;
    ros::Subscriber predictedObjectsSub_;

    lanelet::Id currentLaneletId_ = 0;

    automated_driving_msgs::MotionState egoMotionState_;
    automated_driving_msgs::ObjectStateArray predictedObjectArray_;

    dynamic_reconfigure::Server<PlannerConfig> reconfigSrv_; // Dynamic reconfiguration service

    PlannerInterface params_;

    lanelet::LaneletMapConstPtr mapPtr_;
    lanelet::routing::RoutingGraphPtr routingGraphPtr_;
    lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr_;
    std::string mapFrameId_;
    ros::Time tsLastPlan_;
    std::vector<lanelet::ConstLanelet> laneletVector_;
    bool goalAdded_{false};

    void egoMotionCallback(const automated_driving_msgs::MotionState::ConstPtr& msg);
    void predictedObjectsCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);
    void reconfigureRequest(PlannerConfig&, uint32_t);

    void doPlanning();
};

} // namespace sim_sample_planning_ros_tool
