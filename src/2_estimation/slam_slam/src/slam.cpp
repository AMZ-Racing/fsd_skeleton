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
#include "slam.hpp"
#include <sstream>

namespace ns_slam {
// Constructor
Slam::Slam() {
  initializeState();
};

// Getters
std::vector<fsd_common_msgs::Cone> Slam::getMap() const { return cone_map_; }
geometry_msgs::Pose2D Slam::getState() const { return slam_state_; }

// Setters
void Slam::setMaxMapSize(int size) { max_map_size_ = size; }

void Slam::initializeState() {
  slam_state_.x = 0;
  slam_state_.y = 0;
  slam_state_.theta = 0;
}

void Slam::updateMap(const fsd_common_msgs::ConeDetections &cones) {
  ROS_INFO("Map size is %d", (int) cone_map_.size());
  int excess_cone_length = cone_map_.size() + cones.cone_detections.size() - max_map_size_;
  if (cone_map_.size() + cones.cone_detections.size() > max_map_size_) {
    ROS_INFO("Removing %d elements to accomodate new elements", (int) excess_cone_length);
    cone_map_.erase(cone_map_.begin(), cone_map_.begin() + excess_cone_length);
  }
  for (int i = 0; i < cones.cone_detections.size(); i++) {
    ROS_INFO("cone add to map");
    cone_map_.push_back(cones.cone_detections[i]);
  }
}

void Slam::calculateState(const fsd_common_msgs::CarStateDt &velocity) {
  slam_state_.x += velocity.car_state_dt.x;
  slam_state_.y += velocity.car_state_dt.y;
  slam_state_.theta += velocity.car_state_dt.theta;
}

}
