#include "planner_rviz.hpp"

namespace sim_sample_planning_ros_tool {

PlannerRviz::PlannerRviz(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */
    navMsgsPubEgoView_ = nh_public.advertise<nav_msgs::Path>(params_.path_ego_view_out_topic,
                                                             params_.msg_queue_size);

    navMsgsPubGroundTruth_ = nh_public.advertise<nav_msgs::Path>(
        params_.path_ground_truth_out_topic, params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is
    // received.
    desiredMotionSub_ = nh_public.subscribe(params_.desired_motion_in_topic,
                                            params_.msg_queue_size,
                                            &PlannerRviz::desiredMotionSubCallback,
                                            this,
                                            ros::TransportHints().tcpNoDelay());

    egoMotionSub_ = nh_public.subscribe(params_.ego_motion_in_topic,
                                        params_.msg_queue_size,
                                        &PlannerRviz::egoMotionCallback,
                                        this,
                                        ros::TransportHints().tcpNoDelay());

    objectsStateSub_ = nh_public.subscribe(params_.objects_ground_truth_topic_with_ns,
                                           params_.msg_queue_size,
                                           &PlannerRviz::objectsStateCallback,
                                           this,
                                           ros::TransportHints().tcpNoDelay());

    ROS_INFO_THROTTLE(1,
                      "%s: Test, obj_id= %s)",
                      ros::this_node::getName().c_str(),
                      std::to_string(params_.vehicle_id).c_str());
}

void PlannerRviz::egoMotionCallback(const automated_driving_msgs::MotionState::ConstPtr& msg) {

    egoMotionStateEgoView = *msg;
}

void PlannerRviz::objectsStateCallback(
    const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {

    size_t number_of_objects = msg->objects.size();
    for (size_t i = 0; i < number_of_objects; i++) {
        if (msg->objects[i].object_id == params_.vehicle_id) {
            egoMotionStateGroundTruth = msg->objects[i].motion_state;
        }
    }
}

void PlannerRviz::desiredMotionSubCallback(
    const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& msg) {

    if (msg->object_id == params_.vehicle_id) {

        ros::Time timeNow = ros::Time::now();

        if (&egoMotionStateEgoView) {
            nav_msgs::Path path;
            path.header.stamp = timeNow;
            path.header.frame_id = egoMotionStateEgoView.header.frame_id;

            for (size_t i = 0; i < msg->delta_poses_with_delta_time.size(); i++) {
                geometry_msgs::PoseStamped pathPose;
                geometry_msgs::Pose deltaPose;
                deltaPose = msg->delta_poses_with_delta_time[i].delta_pose;

                ros::Time stamp = ros::Time();
                stamp = timeNow + msg->delta_poses_with_delta_time[i].delta_time;

                pathPose.header.stamp = stamp;
                pathPose.header.frame_id = egoMotionStateEgoView.header.frame_id;

                Eigen::Affine3d startPoseEigen;
                Eigen::Affine3d deltaPoseEigen;
                Eigen::Affine3d newPoseEigen;
                tf::poseMsgToEigen(egoMotionStateEgoView.pose.pose, startPoseEigen);
                tf::poseMsgToEigen(deltaPose, deltaPoseEigen);
                newPoseEigen = startPoseEigen * deltaPoseEigen;
                tf::poseEigenToMsg(newPoseEigen, pathPose.pose);

                path.poses.push_back(pathPose);
            }

            navMsgsPubEgoView_.publish(path);
        }

        if (&egoMotionStateGroundTruth) {
            // TODO
        }
    }
}

} // namespace sim_sample_planning_ros_tool
