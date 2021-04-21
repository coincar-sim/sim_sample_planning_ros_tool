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

#include <simulation_only_msgs/ObjectRemoval.h>
#include <util_eigen_geometry/util_eigen_geometry.hpp>

#include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>


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
    trafficRulesPtr_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
                                                                           lanelet::Participants::Vehicle);
    routingGraphPtr_ = lanelet::routing::RoutingGraph::build(*mapPtr_, *trafficRulesPtr_);


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
        ROS_ERROR_STREAM_THROTTLE(5,
                                  "egoMotionState_.header.frame_id (\"" << egoMotionState_.header.frame_id
                                                                        << "\") != mapFrameId_ (\"" << mapFrameId_
                                                                        << "\"), not planning");
        return;
    }
    tsLastPlan_ = ros::Time::now();
    double desiredPlanningHorizonMeters = params_.v_desired * params_.planning_horizon_secs;

    lanelet::matching::ObjectWithCovariance2d matchingObj;
    matchingObj.pose = util_planner::poseFromMotionState(egoMotionState_);
    matchingObj.positionCovariance = matchingObj.positionCovariance.Identity(); // 1 m
    matchingObj.vonMisesKappa = 1. / (10. / 180. * M_PI);                       // 10 degrees

    auto matches = lanelet::matching::getProbabilisticMatches(*mapPtr_, matchingObj, 1.);
    matches = lanelet::matching::removeNonRuleCompliantMatches(matches, trafficRulesPtr_);

    if (matches.empty()) {
        ROS_ERROR_STREAM_THROTTLE(5, "Could not match ego vehicle to any lanelet, not planning");
        return;
    }

    // Add goal if given and not yet added
    if (!goalAdded_ && params_.lanelet_id_goal != 0L) {
        assert(laneletVector_.empty());
        lanelet::ConstLanelet goalLanelet;
        try {
            goalLanelet = mapPtr_->laneletLayer.get(params_.lanelet_id_goal);
        } catch (lanelet::LaneletError& e) {
            ROS_ERROR_STREAM("Error retrieving goal lanelet with id \"" << params_.lanelet_id_goal << "\" from map:\""
                                                                        << e.what() << "\". Throwing.");
            throw e;
        }

        lanelet::ConstLanelet currentLanelet = matches.at(0).lanelet;
        auto route = routingGraphPtr_->getRoute(currentLanelet, goalLanelet, 0, false); // no lane changes
        lanelet::Optional<lanelet::routing::LaneletPath> laneletPath = route->shortestPath();

        if (!laneletPath.is_initialized()) {
            ROS_ERROR_STREAM("Cannot reach goal lanelet " << goalLanelet.id() << " from current lanelet "
                                                          << currentLanelet.id() << ". Throwing.");
            throw std::runtime_error("Cannot reach goal lanelet " + std::to_string(goalLanelet.id()) +
                                     " from current lanelet " + std::to_string(currentLanelet.id()));
        }

        auto remainingLane = laneletPath->getRemainingLane(laneletPath->front());
        for (const auto& ll : remainingLane) {
            laneletVector_.push_back(ll);
        }

        goalAdded_ = true;
    }

    // Add random route if desired
    if (params_.drive_random_after_goal_reached) {
        if (laneletVector_.empty()) {
            // use best match
            lanelet::ConstLanelet currentLanelet = matches.at(0).lanelet;
            laneletVector_.push_back(currentLanelet);
        }
        while (!routingGraphPtr_->following(laneletVector_.back()).empty()) {
            double length = util_planner::length(laneletVector_);
            if (length > desiredPlanningHorizonMeters && laneletVector_.size() > 1) {
                break;
            }
            laneletVector_.push_back(routingGraphPtr_->following(laneletVector_.back()).front());
        }
    }

    if (laneletVector_.empty()) {
        ROS_ERROR_STREAM(
            "Could not determine route. Probably no goal lanelet given and random drive disabled. Throwing.");
        throw std::runtime_error(
            "Could not determine route. Probably no goal lanelet given and random drive disabled.");
    }

    // Find lanelet match along ego route and remove lanelets that we passed already
    lanelet::ConstLanelet currentLanelet;
    bool matchInVector{false};
    for (const auto& match : matches) {
        auto iter = std::find_if(
            laneletVector_.begin(), laneletVector_.end(), [&](auto& llt) { return llt.id() == match.lanelet.id(); });
        if (iter != laneletVector_.end()) {
            currentLanelet = *iter;
            matchInVector = true;
            if (iter != laneletVector_.begin()) {
                // remove lanelets that we passed already
                laneletVector_.erase(laneletVector_.begin(), iter);
            }
            break;
        }
    }

    if (!matchInVector) {
        ROS_ERROR_STREAM("Could not match object to existing laneletVector. Throwing.");
        throw std::runtime_error("Could not match object to existing laneletVector.");
    }

    assert(currentLanelet.id() == laneletVector_.at(0).id());
    const double currentProjectionOnPath =
        lanelet::geometry::toArcCoordinates(laneletVector_.at(0).centerline2d(), matchingObj.pose.translation()).length;

    // Add random route if desired
    if (params_.drive_random_after_goal_reached) {
        while (!routingGraphPtr_->following(laneletVector_.back()).empty()) {
            double length = util_planner::length(laneletVector_);
            if (length > desiredPlanningHorizonMeters + currentProjectionOnPath && laneletVector_.size() > 1) {
                break;
            }
            laneletVector_.push_back(routingGraphPtr_->following(laneletVector_.back()).front());
        }
    }

    // Check if end of route reached
    if (laneletVector_.size() == 1) {
        ROS_INFO("Final lanelet reached, removing object.");
        simulation_only_msgs::ObjectRemoval objectRemoval;
        objectRemoval.header.stamp = ros::Time::now();
        objectRemoval.object_id = params_.vehicle_id;
        params_.object_removal_pub.publish(objectRemoval);
        ros::shutdown();
    }


    // Path from centerline of laneletVector_
    std::vector<lanelet::BasicPoint2d> pointVec;
    lanelet::BasicLineString2d path;
    for (const auto& ll : laneletVector_) {
        for (const auto& pt : ll.centerline()) {
            path.push_back(pt.basicPoint2d());
        }
    }

    lanelet::BasicLineString2d targetPath;
    for (const auto& pt : path) {
        double positionOfPointAlongPath = lanelet::geometry::toArcCoordinates(path, pt).length;
        if (positionOfPointAlongPath > currentProjectionOnPath) {
            targetPath.push_back(pt);
            if (positionOfPointAlongPath > desiredPlanningHorizonMeters + currentProjectionOnPath) {
                break;
            }
        }
    }


    // Get the resulting trajectory (starting at current pose)
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj = util_planner::deltaTrajFromMotionStateAndPathAndVelocity(
        egoMotionState_, targetPath, params_.v_desired, params_.vehicle_id, ros::Time::now());

    desiredMotionPub_.publish(deltaTraj);
}

} // namespace sim_sample_planning_ros_tool
