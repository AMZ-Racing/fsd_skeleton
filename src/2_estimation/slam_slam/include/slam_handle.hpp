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

#ifndef ESTIMATION_SLAM_SLAM_HANDLE_HPP
#define ESTIMATION_SLAM_SLAM_HANDLE_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "geometry_msgs/Pose2D.h"
#include "slam.hpp"

namespace ns_slam {

class SlamHandle {

 public:
  // Constructor
  SlamHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMap();
  void sendState();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber visionConeDetectionsSubscriber_;
  ros::Subscriber lidarConeDetectionsSubscriber_;
  ros::Subscriber velocityEstimateSubscriber_;
  ros::Publisher slamMapPublisher_;
  ros::Publisher slamStatePublisher_;

  void lidarConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones);
  void visionConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones);
  void velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity);

  std::string vision_cone_detections_topic_name_;
  std::string velocity_estimate_topic_name_;
  std::string lidar_cone_detections_topic_name_;
  std::string slam_map_topic_name_;
  std::string slam_state_topic_name_;
  int node_rate_;

  Slam slam_;
  fsd_common_msgs::ConeDetections cone_detections_;
  fsd_common_msgs::CarStateDt velocity_estimate_;
  fsd_common_msgs::Map slam_map_;
  fsd_common_msgs::CarState slam_state_;

  int max_map_size_;
};
}

#endif //ESTIMATION_SLAM_SLAM_HANDLE_HPP
