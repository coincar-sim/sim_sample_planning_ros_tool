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

#include <util_eigen_geometry/util_eigen_geometry.hpp>

#include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>

#include "planner.hpp"

namespace sim_sample_planning_ros_tool {

Planner::Planner(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    // Get lanelet map
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
    mapPtr_ = ll2if.waitForMapPtr(10, 30);
    mapFrameId_ = ll2if.waitForFrameIdMap(10, 30);

    // Get lanelet routing graph
    lanelet::traffic_rules::TrafficRulesPtr trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    routingGraphPtr_ = lanelet::routing::RoutingGraph::build(*mapPtr_, *trafficRules);


    tsLastPlan_.fromSec(0);

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&Planner::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */
    desiredMotionPub_ = node_handle.advertise<simulation_only_msgs::DeltaTrajectoryWithID>(
        params_.desired_motion_out_topic, params_.msg_queue_size);
    // Instantiate subscriber last, to assure all objects are initialised when first message is
    // received.
    egoMotionSub_ = node_handle.subscribe(params_.ego_motion_in_topic,
                                          params_.msg_queue_size,
                                          &Planner::egoMotionCallback,
                                          this,
                                          ros::TransportHints().tcpNoDelay());
    predictedObjectsSub_ = node_handle.subscribe(params_.predicted_objects_in_topic,
                                                 params_.msg_queue_size,
                                                 &Planner::predictedObjectsCallback,
                                                 this,
                                                 ros::TransportHints().tcpNoDelay());

    if (mapPtr_->laneletLayer.empty()) {
        ROS_ERROR("%s: Lanelet map does not contain any lanelets! Stopping this node!",
                  ros::this_node::getName().c_str());
        node_handle.shutdown();
    }
}


void Planner::egoMotionCallback(const automated_driving_msgs::MotionState::ConstPtr& msg) {

    egoMotionState_ = *msg;
    doPlanning();
}


void Planner::predictedObjectsCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {

    predictedObjectArray_ = *msg;
}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void Planner::reconfigureRequest(PlannerConfig& config, uint32_t level) {
    params_.fromConfig(config);
}

void Planner::doPlanning() {
    if ((ros::Time::now() - tsLastPlan_).toSec() < 1. / params_.max_planning_frequency) {
        return;
    }
    if (egoMotionState_.header.frame_id != mapFrameId_) {
        ROS_ERROR_THROTTLE(5, "egoMotionState_.header.frame_id != mapFrameId_, not planning");
        return;
    }
    tsLastPlan_ = ros::Time::now();
    double planningHorizon = 30.0; // params_.v_desired * 20.0;

    // Call planner function
    util_eigen_geometry::polygon_t targetPath;
    std::tie(targetPath, currentLaneletId_) = util_planner::doPlanningExternal(
        mapPtr_, routingGraphPtr_, egoMotionState_, currentLaneletId_, planningHorizon);

    // get the resulting trajectory (starting at current pose)
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj = util_planner::deltaTrajFromMotionStateAndPathAndVelocity(
        egoMotionState_, targetPath, params_.v_desired, params_.vehicle_id, ros::Time::now());

    desiredMotionPub_.publish(deltaTraj);
}


} // namespace sim_sample_planning_ros_tool
