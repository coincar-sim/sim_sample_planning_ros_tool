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

#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>


namespace util_planner {

simulation_only_msgs::DeltaTrajectoryWithID deltaTrajFromMotionStateAndPathAndVelocity(
    const automated_driving_msgs::MotionState& egoMotionState,
    const util_eigen_geometry::polygon_t& path,
    const double velocity,
    const int32_t objectId,
    const ros::Time& timestamp);

geometry_msgs::Point pointFromVector2d(const Eigen::Vector2d& point);

inline lanelet::matching::Pose2d poseFromMotionState(const automated_driving_msgs::MotionState& ms) {
    Eigen::Isometry3d pose3d;
    util_geometry_msgs::conversions::fromMsg(ms.pose.pose, pose3d);
    return util_eigen_geometry::isometry2dFromXYOfIsometry3d(pose3d);
}

inline double length(const std::vector<lanelet::ConstLanelet>& laneletVector) {
    double length = 0.;
    for (const auto& ll : laneletVector) {
        length += lanelet::geometry::length2d(ll);
    }
    return length;
}

} // namespace util_planner
