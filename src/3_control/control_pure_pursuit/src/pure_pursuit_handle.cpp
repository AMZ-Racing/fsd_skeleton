/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "pure_pursuit_handle.hpp"

namespace ns_pure_pursuit {

// Constructor
PurePursuitHandle::PurePursuitHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
  pure_pursuit_.setMaxSpeed(max_speed_);
}

// Getters
int PurePursuitHandle::getNodeRate() const { return node_rate_; }

// Methods
void PurePursuitHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param("max_speed",
                                      max_speed_,
                                      3.0)) {
    ROS_WARN_STREAM("Did not load max_speed. Standard value is: " << max_speed_);
  }
  if (!nodeHandle_.param<std::string>("slam_map_topic_name",
                                      slam_map_topic_name_,
                                      "/estimation/slam/map")) {
    ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << slam_map_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("slam_state_topic_name",
                                      slam_state_topic_name_,
                                      "/estimation/slam/state")) {
    ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("velocity_estimate_topic_name",
                                      velocity_estimate_topic_name_,
                                      "/estimation/velocity_estimation/velocity_estimate")) {
    ROS_WARN_STREAM("Did not load velocity_estimate_topic_name. Standard value is: " << velocity_estimate_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("control_command_topic_name",
                                      control_command_topic_name_,
                                      "/control/pure_pursuit/car_command")) {
    ROS_WARN_STREAM("Did not load control_command_topic_name. Standard value is: " << control_command_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void PurePursuitHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  slamMapSubscriber_ = nodeHandle_.subscribe(slam_map_topic_name_, 1, &PurePursuitHandle::slamMapCallback, this);
  slamStateSubscriber_ = nodeHandle_.subscribe(slam_state_topic_name_, 1, &PurePursuitHandle::slamStateCallback, this);
  velocityEstimateSubscriber_ = nodeHandle_.subscribe(velocity_estimate_topic_name_, 1, &PurePursuitHandle::velocityEstimateCallback, this);
}

void PurePursuitHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  controlCommandPublisher_ = nodeHandle_.advertise<fsd_common_msgs::ControlCommand>(control_command_topic_name_, 1);
}

void PurePursuitHandle::run() {
  pure_pursuit_.runAlgorithm();
  sendControlCommand();
}

void PurePursuitHandle::sendControlCommand() {
  control_command_.throttle = pure_pursuit_.getControlCommand().throttle;
  control_command_.steering_angle = pure_pursuit_.getControlCommand().steering_angle;
  control_command_.header.stamp = ros::Time::now();
  controlCommandPublisher_.publish(control_command_);
  ROS_INFO("Control command sent");
}

void PurePursuitHandle::slamMapCallback(const fsd_common_msgs::Map &map) {
  pure_pursuit_.setMap(map);
}

void PurePursuitHandle::slamStateCallback(const fsd_common_msgs::CarState &state) {
  pure_pursuit_.setState(state);
}

void PurePursuitHandle::velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity) {
  pure_pursuit_.setVelocity(velocity);
}
}
