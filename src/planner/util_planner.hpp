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

#include <ros/ros.h>

#include <automated_driving_msgs/MotionState.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>

#include <util_eigen_geometry/util_eigen_geometry.hpp>
#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace util_planner {

simulation_only_msgs::DeltaTrajectoryWithID deltaTrajFromMotionStateAndPathAndVelocity(
    const automated_driving_msgs::MotionState& egoMotionState,
    const util_eigen_geometry::polygon_t& path,
    const double velocity,
    const int32_t objectId,
    const ros::Time& timestamp);

geometry_msgs::Point pointFromVector2d(const Eigen::Vector2d& point);

inline bool findExactLaneletMatchesFromPosition(const lanelet::LaneletMapConstPtr& theMapPtr,
                                                const lanelet::BasicPoint2d& position,
                                                lanelet::Ids& foundIds) {
    // Get current lanelets (4 closest ones) from position
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets =
        lanelet::geometry::findNearest(theMapPtr->laneletLayer, position, 4);

    // Find exact matches (current position lies within)
    for (const auto& laneletPair : nearestLanelets) {
        if (laneletPair.first <= 0.0001) {
            foundIds.push_back(laneletPair.second.id());
        }
    }
    if (foundIds.size() != 0) {
        return true;
    } else {
        return false;
    }
}


inline void chooseLaneletIdByOrientation(const lanelet::LaneletMapConstPtr& theMapPtr,
                                         const automated_driving_msgs::MotionState& egoMotionState_,
                                         const lanelet::Ids& exactMatchesLaneletIds,
                                         lanelet::Id& foundId) {
    double highestCosineSimilarity = -1.0;
    Eigen::Isometry3d eigenPose3d;
    util_geometry_msgs::conversions::fromMsg(egoMotionState_.pose.pose, eigenPose3d);
    Eigen::Affine2d eigenPose = util_eigen_geometry::affine2dFromXYOfAffine3d(eigenPose3d);

    for (auto& laneletId : exactMatchesLaneletIds) {
        auto laneletLineString = lanelet::ConstLineString2d(theMapPtr->laneletLayer.get(laneletId).centerline());
        // Transform LineString to polygon type
        util_eigen_geometry::polygon_t laneletPolygon;
        std::transform(
            laneletLineString.begin(), laneletLineString.end(), std::back_inserter(laneletPolygon), [](auto& point) {
                Eigen::Vector2d v{point.x(), point.y()};
                return v;
            });
        double cosineSimilarity = util_eigen_geometry::cosineSimilarity(eigenPose, laneletPolygon);

        if (cosineSimilarity > highestCosineSimilarity) {
            foundId = laneletId;
            highestCosineSimilarity = cosineSimilarity;
        }
    }
}


inline void chooseLaneletIdByPreviousLaneletId(const lanelet::LaneletMapConstPtr& theMapPtr,
                                               const lanelet::routing::RoutingGraphPtr& routingGraphPtr,
                                               const automated_driving_msgs::MotionState& egoMotionState,
                                               const lanelet::Ids& exactMatchesLaneletIds,
                                               lanelet::Id& currentLaneletId) {
    // Check if we are still in the same (previous) lanelet
    if (std::find(exactMatchesLaneletIds.begin(), exactMatchesLaneletIds.end(), currentLaneletId) !=
        exactMatchesLaneletIds.end()) {
        return;
    }

    // Check if we are in a successor of the previous lanelet
    // Get successors of previous lanelet
    lanelet::ConstLanelets followLanelets = routingGraphPtr->following(theMapPtr->laneletLayer.get(currentLaneletId));
    lanelet::Ids followLaneletIds{};
    std::transform(followLanelets.begin(),
                   followLanelets.end(),
                   std::back_inserter(followLaneletIds),
                   [](auto& lanelet) { return lanelet.id(); });
    // Check if we are inside a successor lanelet
    for (lanelet::Id flaneletId : followLaneletIds) {
        if (std::find(exactMatchesLaneletIds.begin(), exactMatchesLaneletIds.end(), flaneletId) !=
            exactMatchesLaneletIds.end()) {
            currentLaneletId = flaneletId;
            return;
        }
    }

    // Check if we are in a successor of a successor of the previous lanelet
    for (lanelet::Id flaneletId : followLaneletIds) {
        lanelet::ConstLanelets ffollowLanelets = routingGraphPtr->following(theMapPtr->laneletLayer.get(flaneletId));
        lanelet::Ids ffollowLaneletIds{};
        std::transform(ffollowLanelets.begin(),
                       ffollowLanelets.end(),
                       std::back_inserter(ffollowLaneletIds),
                       [](auto& lanelet) { return lanelet.id(); });
        for (lanelet::Id fflaneletId : ffollowLaneletIds) {
            if (std::find(exactMatchesLaneletIds.begin(), exactMatchesLaneletIds.end(), fflaneletId) !=
                exactMatchesLaneletIds.end()) {
                currentLaneletId = fflaneletId;
                return;
            }
        }
    }

    // Else, get it by orientation
    chooseLaneletIdByOrientation(theMapPtr, egoMotionState, exactMatchesLaneletIds, currentLaneletId);
    return;
}

inline std::pair<util_eigen_geometry::polygon_t, lanelet::Id> doPlanningExternal(
    const lanelet::LaneletMapConstPtr& theMapPtr,
    const lanelet::routing::RoutingGraphPtr& routingGraphPtr,
    const automated_driving_msgs::MotionState& egoMotionState,
    const lanelet::Id& lastLaneletId,
    const double& planningHorizon) {

    lanelet::Id currentLaneletId = lastLaneletId;
    // Get current position
    lanelet::BasicPoint2d currentPosition{egoMotionState.pose.pose.position.x, egoMotionState.pose.pose.position.y};

    // Get matching lanelets
    lanelet::Ids exactMatchesLaneletIds;
    bool foundIds = findExactLaneletMatchesFromPosition(theMapPtr, currentPosition, exactMatchesLaneletIds);
    if (!foundIds) {
        ROS_ERROR("Could not find lanelets close to given position");
        throw std::runtime_error("Could not find lanelets close to given position");
    }


    // Determine current lanelet
    if (exactMatchesLaneletIds.size() == 1) { // Only one option
        currentLaneletId = exactMatchesLaneletIds.at(0);
    } else { // Ambigious result, need to find correct lanelet
        if (currentLaneletId != 0) {
            // we have a previous currenLanelet and can use it to check following lanelets
            chooseLaneletIdByPreviousLaneletId(
                theMapPtr, routingGraphPtr, egoMotionState, exactMatchesLaneletIds, currentLaneletId);
        } else {
            // Find by orientation
            chooseLaneletIdByOrientation(theMapPtr, egoMotionState, exactMatchesLaneletIds, currentLaneletId);
        }
    }

    // get the path from this sequence
    const lanelet::ConstLanelet curr = theMapPtr->laneletLayer.get(currentLaneletId);
    auto possiblePaths = routingGraphPtr->possiblePaths(curr, planningHorizon, 0, false);
    if (possiblePaths.size() == 0) {
        throw std::runtime_error("Did not find a path that has at least a planning horizon of " +
                                 std::to_string(planningHorizon));
    }

    lanelet::routing::LaneletPath possibleLaneletPath = possiblePaths.front();
    auto possiblePath = lanelet::BasicLineString2d(
        lanelet::CompoundLineString2d(possibleLaneletPath.getRemainingLane(possibleLaneletPath.begin()).centerline())
            .basicLineString());
    // Transform Path to polygon type
    util_eigen_geometry::polygon_t path;
    std::transform(possiblePath.begin(), possiblePath.end(), std::back_inserter(path), [](auto& point) {
        Eigen::Vector2d v{point.x(), point.y()};
        return v;
    });

    // get the resulting target path (starting at current pose)
    util_eigen_geometry::polygon_t targetPath;
    Eigen::Vector2d position2d{egoMotionState.pose.pose.position.x, egoMotionState.pose.pose.position.y};
    size_t closestId = util_eigen_geometry::getClosestId(position2d, path);
    util_eigen_geometry::splitPolygonRight(path, closestId, targetPath);

    return std::make_pair(targetPath, currentLaneletId);
}

} // namespace util_planner
