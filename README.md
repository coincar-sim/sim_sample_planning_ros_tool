# sim_sample_planning_ros_tool
Sample planning module for a vehicle in the simulation framework.

#### planner
* plans a trajectory along the centerline of a lanelet sequence, not collision free!

## Installation
* this package is part of the simulation framework
* see [coincarsim_getting_started](https://github.com/coincar-sim/coincarsim_getting_started) for installation and more details

## Usage
* started within the a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_planning.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework

  * **objects_ground_truth_topic_with_ns**: Topic under which the ground truth states of the objects are received
  * **desired_motion_topic_with_ns**: Topic under which the desired motion of the vehicle is published
  * **perc_egomotion_topic**: Topic for the perceived ego motion state
  * **pred_plan_obj_topic**: Topic for the predicted objects
  * **internal_communication_subns**: Subnamespace for vehicle-internal communication

  * **v_desired**: Desired velocity (in m/s)
  * **const_offset**: Constant offset of the lanelet bound (in m)

* the lanelet2_map is retrieved via the package `lanelet2_interface_ros`

## Contribution
* fork this repo
* use your own planning algorithms for generating a delta trajectory
* ensure that
  * `$(arg desired_motion_topic_with_ns)` is published
  * all internal ROS communication stays within the planning namespace

## Contributors
Nick Engelhardt, Alexander Naumann, Maximilian Naumann

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
