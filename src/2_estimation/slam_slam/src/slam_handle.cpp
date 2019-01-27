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
#include "slam_handle.hpp"

namespace ns_slam {

// Constructor
SlamHandle::SlamHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  slam_.setMaxMapSize(max_map_size_);
  subscribeToTopics();
  publishToTopics();
}

// Getters
int SlamHandle::getNodeRate() const { return node_rate_; }

// Methods
void SlamHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("vision_cone_detections_topic_name",
                                      vision_cone_detections_topic_name_,
                                      "/perception/vision/cone_detections")) {
    ROS_WARN_STREAM("Did not load vision_cone_detections_topic_name. Standard value is: " << vision_cone_detections_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("lidar_cone_detections_topic_name",
                                      lidar_cone_detections_topic_name_,
                                      "/perception/lidar/cone_detections")) {
    ROS_WARN_STREAM("Did not load lidar_cone_detections_topic_name. Standard value is: " << lidar_cone_detections_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("velocity_estimate_topic_name",
                                      velocity_estimate_topic_name_,
                                      "/estimation/velocity_estimation/velocity_estimate")) {
    ROS_WARN_STREAM("Did not load velocity_estimate_topic_name. Standard value is: " << velocity_estimate_topic_name_);
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
  if (!nodeHandle_.param("max_map_size", max_map_size_, 30)) {
    ROS_WARN_STREAM("Did not load max_map_size. Standard value is: " << max_map_size_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void SlamHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  visionConeDetectionsSubscriber_ = nodeHandle_.subscribe(vision_cone_detections_topic_name_, 1, &SlamHandle::visionConeDetectionsCallback, this);
  lidarConeDetectionsSubscriber_ = nodeHandle_.subscribe(lidar_cone_detections_topic_name_, 1, &SlamHandle::lidarConeDetectionsCallback, this);
  velocityEstimateSubscriber_ = nodeHandle_.subscribe(velocity_estimate_topic_name_, 1, &SlamHandle::velocityEstimateCallback, this);
}

void SlamHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  slamMapPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Map>(slam_map_topic_name_, 1);
  slamStatePublisher_ = nodeHandle_.advertise<fsd_common_msgs::CarState>(slam_state_topic_name_, 1);
}

void SlamHandle::run() {
  sendMap();
  sendState();
}

void SlamHandle::sendMap() {
  slam_map_.cone_blue = slam_.getMap();
  slam_map_.header.stamp = ros::Time::now();
  slamMapPublisher_.publish(slam_map_);
  ROS_INFO("SLAM map sent");
}

void SlamHandle::sendState() {
  slam_state_.car_state.x = slam_.getState().x;
  slam_state_.car_state.y = slam_.getState().y;
  slam_state_.car_state.theta = slam_.getState().theta;
  slam_state_.header.stamp = ros::Time::now();
  slamStatePublisher_.publish(slam_state_);
  ROS_INFO("SLAM state sent");
}

void SlamHandle::lidarConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones) {
  ROS_INFO("Updating map with LiDAR cones");
  slam_.updateMap(cones);
}

void SlamHandle::visionConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones) {
  ROS_INFO("Updating map with vision cones");
  slam_.updateMap(cones);
  sendMap();
}

void SlamHandle::velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity) {
  slam_.calculateState(velocity);
}
}
