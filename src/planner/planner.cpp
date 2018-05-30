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

#include "planner.hpp"

namespace sim_sample_planning_ros_tool {

Planner::Planner(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr = std::make_shared<util_geo_coordinates::CoordinateTransformRos>();
    coordinateTransformPtr->waitForInit(params_.navsatfix_topic, 10.);
    theMapPtr_ = std::make_shared<util_lanelet::lanelet_map_wrapper>(params_.lanelet_map_filename, coordinateTransformPtr);
    theMapPtr_->setDebugFolder(params_.debug_directory);
    theMapPtr_->initializeBoundaryPolygons();
    theMapPtr_->calculateOffsetPolygons(LLet::RIGHT, params_.const_offset);
    planner_.setDebugFolder(params_.debug_directory);


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

    if (!theMapPtr_->containsLanelets()) {
        ROS_ERROR("%s: Error loading lanelet map! Filename was \"%s\". Stopping this node!",
                  ros::this_node::getName().c_str(),
                  params_.lanelet_map_filename.c_str());
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
    if ((ros::Time::now()-tsLastPlan_).toSec() < 1./params_.max_planning_frequency) {
        return;
    }
    tsLastPlan_ = ros::Time::now();
    double planningHorizon = 100.0; // params_.v_desired * 20.0;

    // get list of possible lanelets
    std::vector<int32_t> laneletIds = theMapPtr_->getLaneletsFromMotionState(egoMotionState_);
    if (laneletIds.size() == 0) {
        ROS_ERROR("%s: Planner::doPlanning for object_id %s: did not find any lanelet! ",
                  ros::this_node::getName().c_str(),
                  std::to_string(params_.vehicle_id).c_str());
        return;
    }

    // get most likely lanelet
    if (currentLaneletId_ != 0) {
        currentLaneletId_ =
            theMapPtr_->getMostLikelyLaneletIdByPreviousLaneletId(egoMotionState_, currentLaneletId_, laneletIds);
    } else {
        currentLaneletId_ = theMapPtr_->getMostLikelyLaneletIdByOrientation(egoMotionState_, laneletIds);
    }

    // get an arbitrary lanelet sequence
    std::vector<int32_t> laneletSeq = theMapPtr_->getArbitraryLaneletSequence(currentLaneletId_, planningHorizon);

    // get the path from this sequence
    util_eigen_geometry::polygon_t path;
    theMapPtr_->getOffsetPolygon(path, laneletSeq);

    // get the resulting target path (starting at current pose)
    util_eigen_geometry::polygon_t targetPath;
    Eigen::Vector2d position2d{egoMotionState_.pose.pose.position.x, egoMotionState_.pose.pose.position.y};
    size_t closestId =
        util_eigen_geometry::getClosestId(position2d, path);
    util_eigen_geometry::splitPolygonRight(path, closestId, targetPath);

    // get the resulting trajectory (starting at current pose)
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj = planner_.deltaTrajFromMotionStateAndPathAndVelocity(
        egoMotionState_, targetPath, params_.v_desired, params_.vehicle_id, ros::Time::now());

    desiredMotionPub_.publish(deltaTraj);
}

} // namespace sim_sample_planning_ros_tool
