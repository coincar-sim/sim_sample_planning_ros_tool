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

#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include "util_planner.hpp"

namespace util_planner {

simulation_only_msgs::DeltaTrajectoryWithID deltaTrajFromMotionStateAndPathAndVelocity(
    const automated_driving_msgs::MotionState& egoMotionState,
    const util_eigen_geometry::polygon_t& path,
    const double velocity,
    const int32_t objectId,
    const ros::Time& timestamp) {

    Eigen::Isometry3d egoPose3d;
    util_geometry_msgs::conversions::fromMsg(egoMotionState.pose.pose, egoPose3d);
    Eigen::Isometry2d egoPose = util_eigen_geometry::isometry2dFromXYOfIsometry3d(egoPose3d);

    // get the deltaPath (from egoPose view)
    util_eigen_geometry::polygon_t deltaPath;
    Eigen::Vector2d zero;
    zero << 0.0, 0.0;
    deltaPath.push_back(zero);

    for (const Eigen::Vector2d& point : path) {
        Eigen::Vector2d deltaPoint;
        deltaPoint = egoPose.inverse() * point;
        deltaPath.push_back(deltaPoint);
    }

    // get the respective (a) time distances when travelling with v_const and (b) orientations
    std::vector<double> deltaT, yaw;
    deltaT.push_back(0.0);
    yaw.push_back(0.0);
    for (size_t i = 1; i < deltaPath.size(); i++) {
        Eigen::Vector2d deltaVec;
        double ds, dt, yaw_;

        deltaVec = deltaPath[i] - deltaPath[i - 1];
        ds = deltaVec.norm();
        dt = ds / velocity;
        deltaT.push_back(deltaT.back() + dt);

        yaw_ = std::atan2(deltaVec[1], deltaVec[0]);

        yaw.push_back(yaw_);
    }

    // create the respective message (instant orientation changes)
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj;
    deltaTraj.object_id = objectId;
    deltaTraj.header.stamp = timestamp;

    for (size_t i = 0; i < path.size(); i++) {

        // delta_pose with orientation of previous section
        if (i > 0) {
            automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt_p;
            ros::Duration duration;
            dpwdt_p.delta_time = duration.fromSec(deltaT[i]);
            dpwdt_p.delta_pose.position = pointFromVector2d(deltaPath[i]);
            dpwdt_p.delta_pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(yaw[i]);
            deltaTraj.delta_poses_with_delta_time.push_back(dpwdt_p);
        }

        // delta_pose with orientation of next section
        if (i < path.size() - 1) {
            automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt;
            ros::Duration duration;
            dpwdt.delta_time = duration.fromSec(deltaT[i]);
            dpwdt.delta_pose.position = pointFromVector2d(deltaPath[i]);
            dpwdt.delta_pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(yaw[i + 1]);
            deltaTraj.delta_poses_with_delta_time.push_back(dpwdt);
        }
    }

    return deltaTraj;
}


geometry_msgs::Point pointFromVector2d(const Eigen::Vector2d& point) {
    geometry_msgs::Point rosPoint;
    rosPoint.x = point[0];
    rosPoint.y = point[1];
    rosPoint.z = 0.0;
    return rosPoint;
}

} // namespace util_planner
