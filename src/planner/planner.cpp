#include "planner.hpp"

namespace sim_sample_planning_ros_tool {

Planner::Planner(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle}, tfListener_{tfBuffer_},
          theMap_{(params_.fromParamServer(), params_.lanelet_map_filename)} {

    /**
     * Initialization
     */
    planner_.setDebugFolder(params_.debug_directory);
    theMap_.setDebugFolder(params_.debug_directory);
    theMap_.setUtmZoneAndBand(32, 'U');
    theMap_.initializeBoundaryPolygons();
    theMap_.calculateOffsetPolygons(LLet::RIGHT, params_.const_offset);

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

    if (!theMap_.containsLanelets()) {
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
    double planningHorizon = 100.0; // params_.v_desired * 20.0;

    // get list of possible lanelets
    std::vector<int32_t> laneletIds = theMap_.getLaneletsFromMotionState(egoMotionState_);
    if (laneletIds.size() == 0) {
        ROS_ERROR("%s: Planner::doPlanning for object_id %s: did not find any lanelet! ",
                  ros::this_node::getName().c_str(),
                  std::to_string(params_.vehicle_id).c_str());
        return;
    }

    // get most likely lanelet
    if (currentLaneletId_ != 0) {
        currentLaneletId_ =
            theMap_.getMostLikelyLaneletIdByPreviousLaneletId(egoMotionState_, currentLaneletId_, laneletIds);
    } else {
        currentLaneletId_ = theMap_.getMostLikelyLaneletIdByOrientation(egoMotionState_, laneletIds);
    }

    // get an arbitrary lanelet sequence
    std::vector<int32_t> laneletSeq = theMap_.getArbitraryLaneletSequence(currentLaneletId_, planningHorizon);

    // get the path from this sequence
    util_geometry::polygon_t path;
    theMap_.getOffsetPolygon(path, laneletSeq);

    // get the resulting target path (starting at current pose)
    util_geometry::polygon_t targetPath;
    size_t closestId =
        util_geometry::getClosestId(util_geometry::getEigenVector2dFromMotionState(egoMotionState_), path);
    util_geometry::splitPolygonRight(path, closestId, targetPath);

    // get the resulting trajectory (starting at current pose)
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj = planner_.deltaTrajFromMotionStateAndPathAndVelocity(
        egoMotionState_, targetPath, params_.v_desired, params_.vehicle_id, ros::Time::now());

    desiredMotionPub_.publish(deltaTraj);
}

} // namespace sim_sample_planning_ros_tool
