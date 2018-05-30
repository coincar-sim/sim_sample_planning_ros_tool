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

namespace util_planner {

class offset_planner {
public:
    offset_planner();

    void setDebugFolder(std::string foldername);

    simulation_only_msgs::DeltaTrajectoryWithID deltaTrajFromMotionStateAndPathAndVelocity(
        const automated_driving_msgs::MotionState& egoMotionState,
        const util_eigen_geometry::polygon_t& path,
        const double velocity,
        const int32_t objectId,
        const ros::Time& timestamp);

private:
    std::string debugFolder_;
};

geometry_msgs::Point pointFromVector2d(const Eigen::Vector2d& point);

void motionStateToCsvFile(const automated_driving_msgs::MotionState& motionState, std::string filename);

void deltaTrajectoryToCsvFile(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajWithID, std::string filename);

} // namespace util_planner
